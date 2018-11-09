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

#ifndef MAPVIZ_SELECT_SERVICE_DIALOG_H
#define MAPVIZ_SELECT_SERVICE_DIALOG_H

#include <set>
#include <vector>

#include <boost/shared_ptr.hpp>

#include <QDialog>
#include <QMetaType>
#include <QThread>

#include <ros/ros.h>

QT_BEGIN_NAMESPACE
class QLineEdit;
class QListWidget;
class QPushButton;
QT_END_NAMESPACE

// This is ugly, but necessary in order to be able to send a std::vector<std::string>
// via a queued signal/slot connection.
typedef std::vector<std::string> ServiceStringVector;
Q_DECLARE_METATYPE(ServiceStringVector);

namespace mapviz
{
  /**
   * Enumerating services requires making a remote service call; doing this in the GUI thread
   * could cause Mapviz to block and become unresponsive, so it is offloaded to another thread.
   */
  class ServiceUpdaterThread : public QThread
  {
    Q_OBJECT
  public:
    ServiceUpdaterThread(ros::NodeHandle& nh, const std::string& allowed_datatype, QObject* parent) :
      QThread(parent),
      nh_(nh),
      allowed_datatype_(allowed_datatype)
    {
    }
    void run();

  Q_SIGNALS:
    void servicesFetched(ServiceStringVector services);
    void fetchingFailed(const QString error_msg);

  private:
    ros::NodeHandle& nh_;
    const std::string& allowed_datatype_;
  };

  /**
   * Provides a dialog that the user can use to either list all known ROS services
   * or all ROS services that handle a particular type.
   */
  class SelectServiceDialog : public QDialog
  {
    Q_OBJECT
  public:
    /**
     * Convenience function for creating a dialog that will prompt the user to select
     * a service and then return the value.  If no service was selected, the returned
     * value will be empty.
     * @param[in] datatype The type of service to search for; if empty, it will show
     *                     the user a list of all services.
     * @param[in] parent The dialog's parent widget.
     * @return The name of the selected service, or an empty string if there was none.
     */
    static std::string selectService(const std::string& datatype, QWidget* parent=0);

    /**
     * Constructs a new SelectServiceDialog and automatically starts a timer that
     * will refresh the list of services every 5 seconds.
     * @param[in] datatype The type of service to search for; if empty, it will show
     *                     the user a list of all services.
     * @param[in] parent The dialog's parent widget.
     */
    SelectServiceDialog(const std::string& datatype = "", QWidget* parent=0);
    virtual ~SelectServiceDialog();

    /**
     * Set a datatype filter to limit displayed topics based on their
     * types.  If the vector is empty (default), the dialog will display
     * all available topics.
     * @param[in] datatype The type of service to search for.
     */
    void setDatatypeFilter(const std::string& datatype);

    /**
     * Gets the service the user had selected, or an empty string if there was
     * none.
     * @return The selected service.
     */
    std::string selectedService() const;

  private Q_SLOTS:
    /**
     * If no worker thread is currently active, this will start a worker thread
     * that will fetch all of the services matching the known data type.
     */
    void fetchServices();
    /**
     * Updates the list of services displayed to the user based on the list
     * of known services and the current filter value.
     */
    void updateDisplayedServices();
    /**
     * Sets our list of known services.
     */
    void updateKnownServices(ServiceStringVector services);
    /**
     * Displays a message box indicating that there was an error and stops our
     * update timer.
     */
    void displayUpdateError(const QString);

  private:
    std::vector<std::string> filterServices();
    void timerEvent(QTimerEvent *);
    void closeEvent(QCloseEvent *);

    ros::NodeHandle nh_;

    std::string allowed_datatype_;
    std::vector<std::string> displayed_services_;
    std::vector<std::string> known_services_;

    int fetch_services_timer_id_;

    QPushButton *cancel_button_;
    QListWidget *list_widget_;
    QLineEdit *name_filter_;
    QPushButton *ok_button_;
    boost::shared_ptr<ServiceUpdaterThread> worker_thread_;
  };
}

#endif //MAPVIZ_SELECT_SERVICE_DIALOG_H
