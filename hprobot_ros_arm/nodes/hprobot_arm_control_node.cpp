#include <QApplication>
#include <QIcon>
#include "hprobot_arm_control.h"


int main(int argc, char *argv[])
{
  //ros::init(argc, argv, "wlkata_control_gui");
  QApplication a(argc, argv);
  ros::init(argc, argv,"HProbotArmControl");
  HProbotArmControl w;

  // set the window title as the node name
  w.setWindowTitle(QString::fromStdString(
                       ros::this_node::getName()));

                       
  // load the icon from our qrc file and set it as the application icon
  //QIcon icon(":/icons/my_gui_icon.png");
  //w.setWindowIcon(icon);

  w.show();

  return a.exec();
}
