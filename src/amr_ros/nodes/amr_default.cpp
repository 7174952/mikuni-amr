#include "amr_default_gui.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "amr_default_gui");
  QApplication a(argc, argv);

  Amr_Default_Gui w;

  // set the window title
  w.setWindowTitle(QString::fromStdString("AMR Default Window"));
  w.init_config_file(argv[1]);
  w.set_serial_info();
  w.start_om_cart();
  w.start_usbio_board();
  w.showMaximized();
  return a.exec();
}
