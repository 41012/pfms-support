#include <DialogKeyboard.h>
#include <QtWidgets>
#include <drone_object_ros.h>
#include <iostream>

int main(int argc, char **argv) {
  ros::init(argc, argv, "drone_keyboard");

  QApplication app(argc, argv);

  ros::NodeHandle node;
  DroneObjectROS drone(node);
  drone.velMode(true);
  DialogKeyboard dlg_keyboard;
  dlg_keyboard.setDrone(drone);
  dlg_keyboard.show();

  return app.exec();
}
