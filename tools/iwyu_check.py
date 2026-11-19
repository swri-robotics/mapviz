#!/usr/bin/env python3
# Copyright (c) 2026, Southwest Research Institute® (SwRI®)
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#     * Redistributions of source code must retain the above copyright
#       notice, this list of conditions and the following disclaimer.
#     * Redistributions in binary form must reproduce the above copyright
#       notice, this list of conditions and the following disclaimer in the
#       documentation and/or other materials provided with the distribution.
#     * Neither the name of the Southwest Research Institute® (SwRI®) nor the
#       names of its contributors may be used to endorse or promote products
#       derived from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY
# DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
# (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
# ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
# (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
# SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
"""
Run include-what-you-use on C++ files, using the compile commands of a colcon build.

Used as a pre-commit hook (see .pre-commit-config.yaml) and by hand:

    tools/iwyu_check.py file.cpp other.hpp   # check these files
    tools/iwyu_check.py --all                # check every file in the build
    tools/iwyu_check.py --all --summary      # only count the findings per package

A header is checked through the source file with the same name, which IWYU
treats as its "associated" header.  Headers without one are skipped.

The build must export compile commands:

    colcon build --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON

By default the findings are advice and the exit status is 0; --strict exits 1
when there are findings, and --strict-on DISTRO does so only for builds against
that ROS distro.
Silence a finding that is wrong for another distro with `// IWYU pragma: keep`.
Nothing is run (exit 0) when IWYU or the compile commands are missing, so the
hook doesn't get in the way of a fresh clone.
"""

import argparse
import collections
import glob
import json
import os
import re
import shlex
import shutil
import subprocess
import sys
import tempfile

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
MAPPING = os.path.join(REPO, 'tools', 'iwyu.imp')
HEADER_EXTENSIONS = ('.hpp', '.h', '.hh', '.hxx')
# IWYU sometimes suggests the C header for size_t, int32_t and the like.
C_HEADERS = {f'{name}.h': f'c{name}' for name in (
    'assert', 'ctype', 'errno', 'float', 'limits', 'math', 'stddef', 'stdint', 'stdio',
    'stdlib', 'string', 'time')}
# Directories of system C headers, which keep their angle brackets.
C_HEADER_DIRS = ('sys/', 'GL/', 'linux/', 'arpa/', 'netinet/', 'net/', 'asm/')
# Libraries whose headers are included through one umbrella header, as the
# repository already does, instead of the many headers they are split into.
UMBRELLAS = {
    'rclcpp/rclcpp.hpp': ['rclcpp', 'rcl', 'rmw', 'rcutils', 'rcpputils', 'tracetools',
                          'libstatistics_collector'],
    'yaml-cpp/yaml.h': ['yaml-cpp'],
    'json/json.h': ['json'],
    'gtest/gtest.h': ['gtest'],
    'gmock/gmock.h': ['gmock'],
}


def default_build_dir():
    # colcon workspaces keep the repository in <ws>/src/<repo> and build in <ws>/build.
    return os.path.normpath(os.path.join(REPO, '..', '..', 'build'))


def load_compile_commands(build_dir):
    """Merge the compile commands of every package built in build_dir."""
    commands = {}
    pattern = os.path.join(build_dir, '*', 'compile_commands.json')
    for path in sorted(glob.glob(pattern)):
        with open(path) as f:
            for entry in json.load(f):
                source = os.path.realpath(os.path.join(entry['directory'], entry['file']))
                if source.startswith(REPO + os.sep):
                    commands[source] = entry
    return commands


def include_dirs(commands):
    """Return the include directories used by the compile commands, plus /usr/include."""
    dirs = []
    for entry in commands.values():
        args = entry.get('arguments') or shlex.split(entry['command'])
        for i, arg in enumerate(args):
            for flag in ('-I', '-isystem'):
                if arg == flag and i + 1 < len(args):
                    dirs.append(args[i + 1])
                elif arg.startswith(flag) and len(arg) > len(flag):
                    dirs.append(arg[len(flag):])
    dirs.append('/usr/include')
    return list(dict.fromkeys(os.path.normpath(d) for d in dirs if os.path.isdir(d)))


def spellings(name):
    return [f'"{name}"', f'<{name}>']


def qt_mappings(dirs):
    """
    Map Qt's lower-case headers to the forwarding headers that include them.

    qpushbutton.h -> <QPushButton>.  When no forwarding header includes it
    directly, map it to a lower-case header that includes it and has one.  Headers
    that qglobal.h pulls in go to <QtGlobal>.
    """
    lines = []
    for module_dir in dirs:
        module = os.path.basename(module_dir)
        if not re.match(r'^Qt[A-Za-z]+$', module) or not re.search(r'/qt[56]/', module_dir):
            continue
        headers = [h for h in os.listdir(module_dir) if re.match(r'^q[a-z0-9_]+\.h$', h)]
        forwarders = collections.defaultdict(list)
        includers = collections.defaultdict(list)
        includes = collections.defaultdict(list)
        for name in sorted(os.listdir(module_dir)):
            path = os.path.join(module_dir, name)
            if not os.path.isfile(path):
                continue
            with open(path, errors='replace') as f:
                text = f.read()
            included = re.findall(r'#\s*include\s*["<](?:%s/)?(q[a-z0-9_]+\.h)[">]' % module, text)
            if re.match(r'^Q[A-Za-z0-9_]+$', name) and name != module and len(included) == 1:
                forwarders[included[0]].append(name)
            elif name in headers:
                includes[name] = included
                for header in included:
                    includers[header].append(name)
        # Everything qglobal.h pulls in, directly or not.
        under_global, todo = set(), list(includes.get('qglobal.h', []))
        while todo:
            header = todo.pop()
            if header not in under_global:
                under_global.add(header)
                todo += includes.get(header, [])
        for header in headers:
            names = forwarders.get(header, [])
            # qbytearray.h -> <QByteArray>, not every class it happens to declare.
            same = [n for n in names if n.lower() + '.h' == header]
            targets = [(f'<{n}>', 'public') for n in (same or names)]
            if header in under_global:
                # Qt 6 split Qt 5's qglobal.h into headers such as <QtMinMax> and <QOverload>
                # that Qt 5 lacks; <QtGlobal> has them all in both.
                targets = [('<QtGlobal>', 'public')]
            if not targets:
                # Chain to the includer's own mapping.
                targets = [(f'<{module}/{h}>', 'private') for h in includers.get(header, [])
                           if h in forwarders][:1]
            for spelling in spellings(header) + spellings(f'{module}/{header}'):
                for target, visibility in targets:
                    lines.append(f'  {{ include: [{json.dumps(spelling)}, "private", '
                                 f'{json.dumps(target)}, "{visibility}"] }},')
            # A header declaring several classes: QMouseEvent -> <QMouseEvent>, not
            # whichever of qevent.h's forwarding headers comes first.
            if header not in under_global:
                for name in names:
                    if not re.match(r'^Qt[A-Z]', name):
                        lines.append(f'  {{ symbol: ["{name}", "private", "<{name}>", '
                                     '"public"] },')
    return lines


def umbrella_mappings(dirs):
    """Map every header of these libraries to the umbrella header the repository uses."""
    lines = []
    for umbrella, packages in UMBRELLAS.items():
        for include_dir in dirs:
            for package in packages:
                root = os.path.join(include_dir, package)
                if not os.path.isdir(root):
                    continue
                for parent, _, files in os.walk(root):
                    for name in files:
                        header = os.path.relpath(os.path.join(parent, name), include_dir)
                        if header == umbrella or not name.endswith(HEADER_EXTENSIONS):
                            continue
                        for spelling in spellings(header):
                            lines.append(f'  {{ include: [{json.dumps(spelling)}, "private", '
                                         f'"<{umbrella}>", "public"] }},')
    return list(dict.fromkeys(lines))


def opencv_mappings(dirs):
    """Map OpenCV headers to their module's: <opencv2/core/mat.hpp> -> <opencv2/core.hpp>."""
    lines = []
    for include_dir in dirs:
        root = os.path.join(include_dir, 'opencv2')
        if not os.path.isdir(root):
            continue
        for name in sorted(os.listdir(root)):
            module = os.path.join(root, name)
            if not (os.path.isdir(module) and os.path.isfile(module + '.hpp')):
                continue
            # Only what the module header pulls in; opencv2/core/cuda.hpp isn't in core.hpp.
            seen, todo = set(), [f'opencv2/{name}.hpp']
            while todo:
                header = todo.pop()
                path = os.path.join(include_dir, header)
                if header in seen or not os.path.isfile(path):
                    continue
                seen.add(header)
                with open(path, errors='replace') as f:
                    todo += re.findall(r'#\s*include\s*["<](opencv2/[^">]+)[">]', f.read())
            for header in sorted(seen):
                if header.startswith(f'opencv2/{name}/'):
                    for spelling in spellings(header):
                        lines.append(f'  {{ include: [{json.dumps(spelling)}, "private", '
                                     f'"<opencv2/{name}.hpp>", "public"] }},')
    return list(dict.fromkeys(lines))


def interface_mappings(dirs):
    """
    Map generated ROS 2 interface headers to the ones to include.

    <std_msgs/msg/detail/header__struct.hpp> -> <std_msgs/msg/header.hpp>.  (IWYU 0.17
    doesn't support back-references in regex mappings, so they are listed one by one.)
    """
    lines = []
    for include_dir in dirs:
        pattern = os.path.join(include_dir, '*', '*', 'detail', '*__*.hpp')
        for path in glob.glob(pattern):
            header = os.path.relpath(path, include_dir)
            package, kind, _, name = header.split(os.sep)
            public = f'{package}/{kind}/{name.split("__")[0]}.hpp'
            if kind not in ('msg', 'srv', 'action') or not os.path.exists(
                    os.path.join(include_dir, public)):
                continue
            for spelling in spellings(header.replace(os.sep, '/')):
                lines.append(f'  {{ include: [{json.dumps(spelling)}, "private", '
                             f'"<{public}>", "public"] }},')
    return list(dict.fromkeys(lines))


# The header or symbol a mapping line is about.
KEY = r'(?:include|symbol): \[("(?:[^"\\]|\\.)*")'


def write_mappings(path, commands):
    dirs = include_dirs(commands)
    # Leave out headers that tools/iwyu.imp maps itself, so its mappings win.
    with open(MAPPING) as f:
        overridden = set(re.findall(KEY, f.read()))
    generated = (qt_mappings(dirs) + umbrella_mappings(dirs) + opencv_mappings(dirs) +
                 interface_mappings(dirs))
    lines = [line for line in generated
             if re.search(KEY, line).group(1) not in overridden]
    with open(path, 'w') as f:
        f.write('[\n')
        f.write('\n'.join(lines))
        f.write('\n]\n')


def ros_distro(commands):
    """
    Return the ROS distro the build used.

    Found from the include directories in the compile commands: the install prefix
    above <prefix>/include holds ros_environment's record of the distro (merged
    installs such as /opt/ros/<distro>), or a sibling prefix does (isolated colcon
    installs).  $ROS_DISTRO, which isn't set everywhere, is the fallback.
    """
    record = os.path.join('share', 'ros_environment', 'environment', '1.ros_distro.dsv')
    for include_dir in include_dirs(commands):
        parts = include_dir.split(os.sep)
        if 'include' not in parts:
            continue
        prefix = os.sep.join(parts[:parts.index('include')])
        for path in (os.path.join(prefix, record),
                     os.path.join(os.path.dirname(prefix), 'ros_environment', record)):
            if os.path.isfile(path):
                with open(path) as f:
                    match = re.search(r'ROS_DISTRO;(\w+)', f.read())
                if match:
                    return match.group(1)
    return os.environ.get('ROS_DISTRO')


def toolchain_args():
    """
    Return the extra arguments IWYU's clang needs to find the system headers.

    Two packaging problems are worked around when a test compile fails:
    - Ubuntu 22.04's iwyu package doesn't depend on clang 13's own headers
      (stddef.h and friends), so use those of another installed clang.
    - Clang picks the newest GCC directory, which may be a gcc-N runtime without
      libstdc++ headers (Ubuntu 26.04 ships gcc-16 libraries next to the default
      g++-15), so use the one the compiler uses.
    """
    def missing(args):
        # A real file: IWYU 0.17 asserts on source read from stdin.
        with tempfile.TemporaryDirectory() as tmp:
            source = os.path.join(tmp, 'probe.cpp')
            with open(source, 'w') as f:
                f.write('#include <stddef.h>\n#include <type_traits>\n')
            probe = subprocess.run(
                ['include-what-you-use', '-fsyntax-only'] + args + [source],
                capture_output=True, text=True)
        return re.findall(r"'([^']+)' file not found", probe.stderr)

    args = []
    if 'stddef.h' in missing(args):
        def version(path):
            return [int(n) for n in re.findall(r'\d+', path)]
        found = glob.glob('/usr/lib/llvm-*/lib/clang/*/include/stddef.h')
        found += glob.glob('/usr/lib/clang/*/include/stddef.h')
        # Prefer the headers of IWYU's own clang version, else the newest.
        iwyu_version = subprocess.run(['include-what-you-use', '--version'],
                                      capture_output=True, text=True).stdout
        major = re.search(r'clang version (\d+)', iwyu_version)
        if found:
            best = max(found, key=lambda p: (
                bool(major) and version(p)[0] == int(major.group(1)), version(p)))
            args.append('-resource-dir=' + os.path.dirname(os.path.dirname(best)))
    if missing(args):
        libgcc = subprocess.run(['c++', '-print-libgcc-file-name'], capture_output=True,
                                text=True).stdout.strip()
        if libgcc:
            args.append('--gcc-install-dir=' + os.path.dirname(libgcc))
    return args


def to_sources(files, commands):
    """Map each file to the source file IWYU should compile to check it."""
    sources = set()
    for name in files:
        path = os.path.realpath(name)
        if path in commands:
            sources.add(path)
        elif path.endswith(HEADER_EXTENSIONS):
            stem = os.path.splitext(os.path.basename(path))[0]
            package = os.path.relpath(path, REPO).split(os.sep)[0]
            for source in commands:
                if (os.path.splitext(os.path.basename(source))[0] == stem and
                        os.path.relpath(source, REPO).split(os.sep)[0] == package):
                    sources.add(source)
    return sorted(sources)


def restyle(line):
    """Write a suggested include the way the repository does (cpplint and ROS style)."""
    match = re.match(r'^#include ([<"])([^>"]+)[>"](.*)$', line)
    if not match:
        return line
    quote, header, rest = match.groups()
    header = C_HEADERS.get(header, header)
    # Package headers: <pkg/name.hpp>, but "pkg/name.h", since cpplint takes any
    # <....h> for a C system header.  Generated ui_*.h files keep their quotes.
    if '/' in header and not header.startswith(C_HEADER_DIRS):
        quote = '"' if header.endswith('.h') else '<'
    if quote == '"' and re.match(r'^ui_\w+\.h$', header):
        # uic's generated headers have no directory.
        rest = '  // NOLINT(build/include_subdir)'
    return f'#include {quote}{header}{">" if quote == "<" else chr(34)}{rest}'


def parse(output):
    """Return {file: (add lines, remove lines)} for files IWYU wants changed."""
    findings = {}
    section = None
    for line in output.splitlines():
        match = re.match(r'^(.*) should (add|remove) these lines:$', line)
        if match:
            path, kind = match.groups()
            section = findings.setdefault(os.path.realpath(path), ([], []))[kind == 'remove']
        elif line.startswith('The full include-list') or line == '---' or not line:
            section = None
        elif section is not None:
            section.append(restyle(line))
    for add, remove in findings.values():
        # The same header with other quotes, as restyle() writes it: nothing to change.
        same = {header_name(line) for line in add} & {header_name(line) for line in remove}
        add[:] = [line for line in add if header_name(line) not in same]
        remove[:] = [line for line in remove if header_name(line) not in same]
    return {path: change for path, change in findings.items()
            if (change[0] or change[1]) and path.startswith(REPO + os.sep)}


def header_name(line):
    match = re.search(r'#include [<"]([^>"]+)[>"]', line)
    return match.group(1) if match else line


def main():
    parser = argparse.ArgumentParser(description=__doc__.split('\n\n')[0])
    parser.add_argument('files', nargs='*', help='C++ files to check')
    parser.add_argument('--all', action='store_true', help='check every file in the build')
    parser.add_argument('--build-dir', default=os.environ.get('IWYU_BUILD_DIR'),
                        help='colcon build directory (default: $IWYU_BUILD_DIR or '
                        '<workspace>/build)')
    parser.add_argument('--strict', action='store_true', help='exit 1 when there are findings')
    parser.add_argument('--strict-on', metavar='DISTRO', action='append', default=[],
                        help='--strict when the build is for this ROS distro (IWYU versions '
                        'differ between distros, so only one of them should fail commits)')
    parser.add_argument('--summary', action='store_true',
                        help='print only the number of findings per package')
    parser.add_argument('-j', '--jobs', type=int, default=os.cpu_count())
    args = parser.parse_args()

    if not shutil.which('include-what-you-use') or not shutil.which('iwyu_tool'):
        print('iwyu_check: include-what-you-use is not installed, skipping '
              '(apt install iwyu)')
        return 0
    build_dir = args.build_dir or default_build_dir()
    commands = load_compile_commands(build_dir)
    if not commands:
        print(f'iwyu_check: no compile_commands.json under {build_dir}, skipping.\n'
              '  Build with: colcon build --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON\n'
              '  or set IWYU_BUILD_DIR to the colcon build directory.')
        return 0

    args.strict = args.strict or ros_distro(commands) in args.strict_on

    sources = sorted(commands) if args.all else to_sources(args.files, commands)
    if not sources:
        return 0

    with tempfile.TemporaryDirectory() as tmp:
        generated = os.path.join(tmp, 'generated.imp')
        write_mappings(generated, commands)
        iwyu_args = toolchain_args() + [
            '-Xiwyu', '--mapping_file=' + MAPPING, '-Xiwyu', '--mapping_file=' + generated,
            '-Xiwyu', '--no_fwd_decls']
        with open(os.path.join(tmp, 'compile_commands.json'), 'w') as f:
            json.dump([commands[s] for s in sources], f)
        result = subprocess.run(
            ['iwyu_tool', '-j', str(args.jobs), '-p', tmp] + sources + ['--'] + iwyu_args,
            capture_output=True, text=True)
    output = result.stdout + result.stderr
    errors = [line for line in output.splitlines()
              if ' error: ' in line or 'Assertion failed' in line]
    findings = parse(output)

    if args.summary:
        per_package = collections.Counter()
        lines = collections.Counter()
        for path, (add, remove) in findings.items():
            package = os.path.relpath(path, REPO).split(os.sep)[0]
            per_package[package] += 1
            lines[package] += len(add) + len(remove)
        for package in sorted(per_package):
            print(f'{package}: {per_package[package]} files, {lines[package]} lines to change')
    else:
        for path, (add, remove) in sorted(findings.items()):
            print(f'{os.path.relpath(path, REPO)}:')
            for line in add:
                print(f'  + {line}')
            for line in remove:
                print(f'  {line}')
    for line in errors:
        print(line)
    if findings or errors:
        print(f'iwyu_check: {len(findings)} of the checked files could use include changes'
              f'{" (advice only)" if not args.strict else ""}; {len(errors)} errors.')
        if args.strict and findings:
            print('  Make these changes, or end an include line that another ROS distro needs '
                  'with\n  `// IWYU pragma: keep`.')
    return 1 if args.strict and (findings or errors) else 0


if __name__ == '__main__':
    sys.exit(main())
