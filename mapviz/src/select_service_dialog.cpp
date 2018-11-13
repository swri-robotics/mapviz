// *****************************************************************************
//
// Copyright (c) 2015, Southwest Research Institute® (SwRI®)
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//     * Redistributions of source code must retain the above copyright
//       notice, this list of conditions and the following disclaimer.
//     * Redistributions in binary form must reproduce the above copyright
//       notice, this list of conditions and the following disclaimer in the
//       documentation and/or other materials provided with the distribution.
//     * Neither the name of Southwest Research Institute® (SwRI®) nor the
//       names of its contributors may be used to endorse or promote products
//       derived from this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL Southwest Research Institute® BE LIABLE 
// FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL 
// DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR 
// SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER 
// CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT 
// LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY 
// OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH
// DAMAGE.
//
// *****************************************************************************

#include <mapviz/select_service_dialog.h>

#include <QCloseEvent>
#include <QHBoxLayout>
#include <QLabel>
#include <QLineEdit>
#include <QListWidget>
#include <QMessageBox>
#include <QPushButton>
#include <QTimerEvent>
#include <QVBoxLayout>

#include <rosapi/Services.h>
#include <rosapi/ServicesForType.h>

namespace mapviz
{
  void ServiceUpdaterThread::run()
  {
    ros::ServiceClient client;

    if (allowed_datatype_.empty())
    {
      client = nh_.serviceClient<rosapi::Services>("/rosapi/services");
    }
    else
    {
      client = nh_.serviceClient<rosapi::ServicesForType>("/rosapi/services_for_type");
    }

    if (!client.waitForExistence(ros::Duration(1)))
    {
      // Check to see whether the rosapi services are actually running.
      Q_EMIT fetchingFailed(tr("Unable to list ROS services.  Is rosapi_node running?"));
      return;
    }

    if (allowed_datatype_.empty())
    {
      rosapi::Services srv;

      ROS_DEBUG("Listing all services.");
      if (client.call(srv))
      {
        Q_EMIT servicesFetched(srv.response.services);
      }
    }
    else {
      rosapi::ServicesForType srv;
      srv.request.type = allowed_datatype_;

      ROS_DEBUG("Listing services for type %s", srv.request.type.c_str());
      if (client.call(srv))
      {
        Q_EMIT servicesFetched(srv.response.services);
      }
      else
      {
        // If there are any dead or unreachable nodes that provide services, even if
        // they're not of the service type we're looking for, the services_for_type
        // service will have an error and not return anything.  Super annoying.
        Q_EMIT fetchingFailed(tr("Unable to list ROS services.  You may have " \
                              "dead nodes; try running \"rosnode cleanup\"."));
      }
    }
  }

  std::string SelectServiceDialog::selectService(const std::string& datatype, QWidget* parent)
  {
    SelectServiceDialog dialog(datatype, parent);
    dialog.setDatatypeFilter(datatype);
    if (dialog.exec() == QDialog::Accepted) {
      return dialog.selectedService();
    } else {
      return "";
    }
  }

  SelectServiceDialog::SelectServiceDialog(const std::string& datatype, QWidget* parent)
      :
      QDialog(parent),
      allowed_datatype_(datatype),
      cancel_button_(new QPushButton("&Cancel")),
      list_widget_(new QListWidget()),
      name_filter_(new QLineEdit()),
      ok_button_(new QPushButton("&Ok"))
  {
    QHBoxLayout *filter_box = new QHBoxLayout();
    filter_box->addWidget(new QLabel("Filter:"));
    filter_box->addWidget(name_filter_);

    QHBoxLayout *button_box = new QHBoxLayout();
    button_box->addStretch(1);
    button_box->addWidget(cancel_button_);
    button_box->addWidget(ok_button_);

    QVBoxLayout *vbox = new QVBoxLayout();
    vbox->addWidget(list_widget_);
    vbox->addLayout(filter_box);
    vbox->addLayout(button_box);
    setLayout(vbox);

    // This is ugly, but necessary in order to be able to send a std::vector<std::string>
    // via a queued signal/slot connection.
    qRegisterMetaType<ServiceStringVector>("ServiceStringVector");

    connect(ok_button_, SIGNAL(clicked(bool)),
            this, SLOT(accept()));
    connect(cancel_button_, SIGNAL(clicked(bool)),
            this, SLOT(reject()));
    connect(name_filter_, SIGNAL(textChanged(const QString &)),
            this, SLOT(updateDisplayedServices()));

    ok_button_->setDefault(true);

    setWindowTitle("Select service...");

    fetch_services_timer_id_ = startTimer(5000);
    fetchServices();
  }

  SelectServiceDialog::~SelectServiceDialog()
  {
    if (worker_thread_)
    {
      // If the thread's parent is destroyed before the thread has finished,
      // it will cause a segmentation fault.  We'll wait a few seconds for
      // it to finish cleanly, and if that doesn't work, try to force it to
      // die and wait a few more.
      worker_thread_->wait(5000);
      if (worker_thread_->isRunning())
      {
        worker_thread_->terminate();
        worker_thread_->wait(2000);
      }
    }
  }

  void SelectServiceDialog::fetchServices()
  {
    // If we don't currently have a worker thread or the previous one has
    // finished, start a new one.
    if (!worker_thread_ || worker_thread_->isFinished())
    {
      worker_thread_.reset(new ServiceUpdaterThread(nh_, allowed_datatype_, this));
      QObject::connect(worker_thread_.get(),
                       SIGNAL(servicesFetched(ServiceStringVector)),
                       this,
                       SLOT(updateKnownServices(ServiceStringVector)));
      QObject::connect(worker_thread_.get(),
                       SIGNAL(fetchingFailed(const QString)),
                       this,
                       SLOT(displayUpdateError(const QString)));
      worker_thread_->start();
    }
  }

  void SelectServiceDialog::updateKnownServices(ServiceStringVector services)
  {
    known_services_ = services;
    updateDisplayedServices();
  }

  void SelectServiceDialog::displayUpdateError(const QString error_msg)
  {
    killTimer(fetch_services_timer_id_);
    QMessageBox mbox(this->parentWidget());
    mbox.setIcon(QMessageBox::Warning);
    mbox.setText(error_msg);
    mbox.exec();
  }

  std::vector<std::string> SelectServiceDialog::filterServices()
  {
    std::vector<std::string> filtered_services;

    QString filter_text = name_filter_->text();

    Q_FOREACH(const std::string& service, known_services_)
    {
      if (QString::fromStdString(service).contains(filter_text, Qt::CaseInsensitive))
      {
        filtered_services.push_back(service);
      }
    }

    return filtered_services;
  }

  void SelectServiceDialog::updateDisplayedServices()
  {
    std::vector<std::string> next_displayed_services = filterServices();

    // It's a lot more work to keep track of the additions/removals like
    // this compared to resetting the QListWidget's items each time, but
    // it allows Qt to properly track the selection and current items
    // across updates, which results in much less frustration for the user.

    std::set<std::string> prev_names;
    for (size_t i = 0; i < displayed_services_.size(); i++) {
      prev_names.insert(displayed_services_[i]);
    }

    std::set<std::string> next_names;
    for (size_t i = 0; i < next_displayed_services.size(); i++) {
      next_names.insert(next_displayed_services[i]);
    }

    std::set<std::string> added_names;
    std::set_difference(next_names.begin(), next_names.end(),
                        prev_names.begin(), prev_names.end(),
                        std::inserter(added_names, added_names.end()));

    std::set<std::string> removed_names;
    std::set_difference(prev_names.begin(), prev_names.end(),
                        next_names.begin(), next_names.end(),
                        std::inserter(removed_names, removed_names.end()));

    // Remove all the removed names
    size_t removed = 0;
    for (size_t i = 0; i < displayed_services_.size(); i++) {
      if (removed_names.count(displayed_services_[i]) == 0) {
        continue;
      }

      QListWidgetItem *item = list_widget_->takeItem(i - removed);
      delete item;
      removed++;
    }

    // Now we can add the new items.
    for (size_t i = 0; i < next_displayed_services.size(); i++) {
      if (added_names.count(next_displayed_services[i]) == 0) {
        continue;
      }

      list_widget_->insertItem(i, QString::fromStdString(next_displayed_services[i]));
      if (list_widget_->count() == 1) {
        list_widget_->setCurrentRow(0);
      }
    }

    displayed_services_.swap(next_displayed_services);
  }

  void SelectServiceDialog::setDatatypeFilter(const std::string& datatype)
  {
    allowed_datatype_ = datatype;
    updateDisplayedServices();
  }

  std::string SelectServiceDialog::selectedService() const
  {
    QModelIndex qt_selection = list_widget_->selectionModel()->currentIndex();

    if (qt_selection.isValid()) {
      int row = qt_selection.row();
      if (row < static_cast<int>(displayed_services_.size())) {
        return displayed_services_[row];
      }
    }

    return "";
  }

  void SelectServiceDialog::timerEvent(QTimerEvent* event)
  {
      if (event->timerId() == fetch_services_timer_id_) {
        fetchServices();
      }
  }

  void SelectServiceDialog::closeEvent(QCloseEvent* event)
  {
    // We don't need to keep making requests from the ROS master.
    killTimer(fetch_services_timer_id_);
    QDialog::closeEvent(event);
  }
}
