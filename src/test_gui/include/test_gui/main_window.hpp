/**
 * @file /include/test_gui/main_window.hpp
 *
 * @brief Qt based gui for test_gui.
 **/
#ifndef test_gui_MAIN_WINDOW_H
#define test_gui_MAIN_WINDOW_H

#include <QMainWindow>
#include <QImage>
#include <QPixmap>
#include <QTimer>
#include <QSpinBox>
#include <QSlider>
#include <QLabel>
#include <QRadioButton>
#include <QCheckBox>
#include <QPushButton>
#include <QSignalBlocker>
#include <QIcon>
#include "qnode.hpp"
#include "ui_mainwindow.h"

class MainWindow : public QMainWindow
{
  Q_OBJECT

public:
  MainWindow(QWidget *parent = nullptr);
  ~MainWindow();
  QNode *qnode;

private:
  Ui::MainWindowDesign *ui;
  void closeEvent(QCloseEvent *event);

  void showImage(QLabel *label, const QImage &img);
  void queueParam(const QString &name, int v);   // 50ms 묶음 전송

  // HSV (대상별)
  struct HsvWidget
  {
    QSlider *slider;
    QLabel *label;
  };
  void loadTargetToSliders();
  QMap<QString, HsvWidget> hsv_widgets_;
  QMap<QString, int> hsv_cache_;
  QString current_target_ = "white";

  // 전역 파라미터 (bev., morph., skel., path.)
  struct GlobalWidget
  {
    QSlider *slider = nullptr;
    QLabel *label = nullptr;
    QSpinBox *spin = nullptr;
  };
  void applyGlobalParam(const QString &name, int v);
  QMap<QString, GlobalWidget> global_widgets_;

  // 상태 표시
  void showPathSource(const QString &src);
  void showMission();
  int turn_ = 0;
  bool parking_ = false;
  QTimer status_timeout_;                         // 경로 출처가 1초 넘게 안 오면 회색

  QString paramFile() const;                      // 저장 파일 경로

  // 공통
  QMap<QString, int> pending_;
  QTimer send_timer_;
  QTimer param_poll_timer_;
};

#endif // test_gui_MAIN_WINDOW_H
