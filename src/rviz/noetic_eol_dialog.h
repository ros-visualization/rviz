#ifndef RVIZ_NOETIC_EOL_DIALOG_H
#define RVIZ_NOETIC_EOL_DIALOG_H

#include <QMessageBox>

namespace rviz
{
class NoeticEOLDialog : public QMessageBox
{
  Q_OBJECT
public:
  NoeticEOLDialog(QWidget* parent = nullptr);
};

} // end namespace rviz

#endif // RVIZ_WAIT_FOR_MASTER_DIALOG_H
