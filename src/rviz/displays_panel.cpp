/*
 * Copyright (c) 2012, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include <QAction>
#include <QTimer>
#include <QHBoxLayout>
#include <QVBoxLayout>
#include <QPushButton>
#include <QInputDialog>
#include <QApplication>
#include <QProgressDialog>

#include <boost/bind/bind.hpp>

#include <rviz/display_factory.h>
#include <rviz/display.h>
#include <rviz/add_display_dialog.h>
#include <rviz/properties/property.h>
#include <rviz/properties/property_tree_widget.h>
#include <rviz/properties/property_tree_with_help.h>
#include <rviz/visualization_manager.h>

#include <rviz/displays_panel.h>

namespace rviz
{
DisplaysPanel::DisplaysPanel(QWidget* parent) : Panel(parent)
{
  tree_with_help_ = new PropertyTreeWithHelp;
  property_grid_ = tree_with_help_->getTree();

  QPushButton* add_button = new QPushButton("Add");
  add_button->setShortcut(QKeySequence(QString("Ctrl+N")));
  add_button->setToolTip("Add a new display, Ctrl+N");
  duplicate_button_ = new QPushButton("Duplicate");
  duplicate_button_->setShortcut(QKeySequence(QString("Ctrl+D")));
  duplicate_button_->setToolTip("Duplicate a display, Ctrl+D");
  duplicate_button_->setEnabled(false);
  remove_button_ = new QPushButton("Remove");
  remove_button_->setShortcut(QKeySequence(QString("Ctrl+X")));
  remove_button_->setToolTip("Remove displays, Ctrl+X");
  remove_button_->setEnabled(false);
  rename_button_ = new QPushButton("Rename");
  rename_button_->setShortcut(QKeySequence(QString("Ctrl+R")));
  rename_button_->setToolTip("Rename a display, Ctrl+R");
  rename_button_->setEnabled(false);

  QHBoxLayout* button_layout = new QHBoxLayout;
  button_layout->addWidget(add_button);
  button_layout->addWidget(duplicate_button_);
  button_layout->addWidget(remove_button_);
  button_layout->addWidget(rename_button_);
  button_layout->setContentsMargins(2, 0, 2, 2);

  QVBoxLayout* layout = new QVBoxLayout;
  layout->setContentsMargins(0, 0, 0, 2);
  layout->addWidget(tree_with_help_);
  layout->addLayout(button_layout);

  setLayout(layout);

  connect(add_button, &QPushButton::clicked, this, &DisplaysPanel::onNewDisplay);
  connect(duplicate_button_, &QPushButton::clicked, this, &DisplaysPanel::onDuplicateDisplay);
  connect(remove_button_, &QPushButton::clicked, this, &DisplaysPanel::onDeleteDisplay);
  connect(rename_button_, &QPushButton::clicked, this, &DisplaysPanel::onRenameDisplay);
  connect(property_grid_, &PropertyTreeWidget::selectionHasChanged, this,
          &DisplaysPanel::onSelectionChanged);

  // additionally to buttons, allow shortcuts F2 / Del to rename / remove displays
  rename_action_ = new QAction("Rename", this);
  rename_action_->setShortcut(QKeySequence("F2"));
  rename_action_->setShortcutContext(Qt::WidgetWithChildrenShortcut);
  rename_action_->setEnabled(false);
  tree_with_help_->addAction(rename_action_);
  remove_action_ = new QAction("Remove", this);
  remove_action_->setShortcut(QKeySequence("Del"));
  remove_action_->setShortcutContext(Qt::WidgetWithChildrenShortcut);
  remove_action_->setEnabled(false);
  tree_with_help_->addAction(remove_action_);

  connect(rename_action_, &QAction::triggered, this, &DisplaysPanel::onRenameDisplay);
  connect(remove_action_, &QAction::triggered, this, &DisplaysPanel::onDeleteDisplay);
}

DisplaysPanel::~DisplaysPanel()
{
}

void DisplaysPanel::onInitialize()
{
  property_grid_->setModel(vis_manager_->getDisplayTreeModel());
}

void DisplaysPanel::onNewDisplay()
{
  QString lookup_name;
  QString display_name;
  QString topic;
  QString datatype;

  QStringList empty;

  QApplication::setOverrideCursor(QCursor(Qt::WaitCursor));
  AddDisplayDialog dialog(vis_manager_->getDisplayFactory(), "Display", empty, empty, &lookup_name,
                          &display_name, &topic, &datatype);
  QApplication::restoreOverrideCursor();

  vis_manager_->stopUpdate();
  if (dialog.exec() == QDialog::Accepted)
  {
    Display* disp = vis_manager_->createDisplay(lookup_name, display_name, true);
    if (!topic.isEmpty() && !datatype.isEmpty())
    {
      disp->setTopic(topic, datatype);
    }
  }
  vis_manager_->startUpdate();
  activateWindow(); // Force keyboard focus back on main window.
}

void DisplaysPanel::onDuplicateDisplay()
{
  QList<Display*> displays_to_duplicate = property_grid_->getSelectedObjects<Display>();
  QList<Display*> duplicated_displays;
  QProgressDialog progress_dlg("Duplicating displays...", "Cancel", 0, displays_to_duplicate.size(),
                               this);
  vis_manager_->stopUpdate();
  progress_dlg.setWindowModality(Qt::WindowModal);
  progress_dlg.show();
  QApplication::processEvents(); // explicitly progress events for update

  // duplicate all selected displays
  for (int i = 0; i < displays_to_duplicate.size(); i++)
  {
    // initialize display
    QString lookup_name = displays_to_duplicate[i]->getClassId();
    QString display_name = displays_to_duplicate[i]->getName();
    Display* disp = vis_manager_->createDisplay(lookup_name, display_name, true);
    // duplicate config
    Config config;
    displays_to_duplicate[i]->save(config);
    disp->load(config);
    duplicated_displays.push_back(disp);
    progress_dlg.setValue(i + 1);
    QApplication::processEvents(); // explicitly progress events for update
    // push cancel to stop duplicate
    if (progress_dlg.wasCanceled())
      break;
  }
  // make sure the newly duplicated displays are selected.
  if (!duplicated_displays.empty())
  {
    QModelIndex first = property_grid_->getModel()->indexOf(duplicated_displays.front());
    QModelIndex last = property_grid_->getModel()->indexOf(duplicated_displays.back());
    QItemSelection selection(first, last);
    property_grid_->selectionModel()->select(selection, QItemSelectionModel::ClearAndSelect);
  }
  vis_manager_->startUpdate();
  activateWindow(); // Force keyboard focus back on main window.
}

void DisplaysPanel::onDeleteDisplay()
{
  QList<Display*> displays_to_delete = property_grid_->getSelectedObjects<Display>();

  for (int i = 0; i < displays_to_delete.size(); i++)
  {
    // Displays can emit signals from other threads with self pointers.  We're
    // freeing the display now, so ensure no one is listening to those signals.
    displays_to_delete[i]->disconnect();
    // Remove dipslay from property tree to avoid memory access after deletion
    displays_to_delete[i]->getParent()->takeChild(displays_to_delete[i]);
    // Delete display later in case there are pending signals to it.
    displays_to_delete[i]->deleteLater();
  }
  // Select new current index
  const QModelIndex& cur = property_grid_->currentIndex();
  QItemSelection selection(cur.sibling(cur.row(), 0),
                           cur.sibling(cur.row(), cur.model()->columnCount() - 1));
  property_grid_->selectionModel()->select(selection, QItemSelectionModel::ClearAndSelect);

  vis_manager_->notifyConfigChanged();
}

void DisplaysPanel::onSelectionChanged()
{
  QList<Display*> displays = property_grid_->getSelectedObjects<Display>();

  int num_displays_selected = displays.size();

  duplicate_button_->setEnabled(num_displays_selected > 0);
  remove_button_->setEnabled(num_displays_selected > 0);
  rename_button_->setEnabled(num_displays_selected == 1);

  remove_action_->setEnabled(num_displays_selected > 0);
  rename_action_->setEnabled(num_displays_selected == 1);
}

void DisplaysPanel::onRenameDisplay()
{
  QList<Display*> displays = property_grid_->getSelectedObjects<Display>();
  if (displays.empty())
  {
    return;
  }
  Display* display_to_rename = displays[0];

  QString old_name = display_to_rename->getName();
  QString new_name =
      QInputDialog::getText(this, "Rename Display", "New Name?", QLineEdit::Normal, old_name);

  if (new_name.isEmpty() || new_name == old_name)
  {
    return;
  }

  display_to_rename->setName(new_name);
}

void DisplaysPanel::save(Config config) const
{
  Panel::save(config);
  tree_with_help_->save(config);
}

void DisplaysPanel::load(const Config& config)
{
  Panel::load(config);
  tree_with_help_->load(config);
}

} // namespace rviz
