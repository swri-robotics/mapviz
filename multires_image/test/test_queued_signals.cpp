// *****************************************************************************
//
// Copyright (c) 2026, Southwest Research Institute® (SwRI®)
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//     * Redistributions of source code must retain the above copyright
//       notice, this list of conditions and the following disclaimer.
//     * Redistributions in binary form must reproduce the above copyright
//       notice, this list of conditions and the following disclaimer in the
//       documentation and/or other materials provided with the distribution.
//     * Neither the name of the Southwest Research Institute® (SwRI®) nor the
//       names of its contributors may be used to endorse or promote products
//       derived from this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY
// DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
// (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
// ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//
// *****************************************************************************
#include <gtest/gtest.h>
#include <QMetaMethod>
#include <QMetaObject>
#include <QMetaType>

#include <multires_image/QGLMap.hpp>
#include <multires_image/tile_cache.hpp>

namespace
{
/// Whether Qt can queue argument @p index of @p method across threads.
bool CanQueue(const QMetaMethod & method, int index)
{
#if QT_VERSION >= 0x060000
  // moc records the argument's actual type.
  return method.parameterMetaType(index).isValid();
#else
  // Qt 5 looks the type up by name, and passes an unregistered pointer along
  // as a void pointer.
  return method.parameterType(index) != QMetaType::UnknownType ||
         method.parameterTypes()[index].endsWith('*');
#endif
}

/// Every signal @p meta_object declares whose arguments can't be queued.
::testing::AssertionResult SignalsCanBeQueued(const QMetaObject & meta_object)
{
  ::testing::AssertionResult result = ::testing::AssertionSuccess();
  for (int i = meta_object.methodOffset(); i < meta_object.methodCount(); i++) {
    const QMetaMethod method = meta_object.method(i);
    if (method.methodType() != QMetaMethod::Signal) {
      continue;
    }
    for (int j = 0; j < method.parameterCount(); j++) {
      if (!CanQueue(method, j)) {
        result = ::testing::AssertionFailure();
        result << method.methodSignature().constData() << " can't queue " <<
          method.parameterTypes()[j].constData() << "; ";
      }
    }
  }
  return result;
}
}  // namespace

TEST(QueuedSignals, TileCacheSignalsCanCrossThreads)
{
  // The cache emits from its loading threads.  Qt 5 can't queue an int64_t,
  // so the texture memory size never reached the viewer there.
  EXPECT_TRUE(SignalsCanBeQueued(multires_image::TileCache::staticMetaObject));
}

TEST(QueuedSignals, ViewerSignalsCanCrossThreads)
{
  EXPECT_TRUE(SignalsCanBeQueued(multires_image::QGLMap::staticMetaObject));
}
