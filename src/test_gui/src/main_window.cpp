/**
 * @file /src/main_window.cpp
 *
 * @brief Implementation for the qt gui.
 **/
#include "../include/test_gui/main_window.hpp"

#include <QDir>

MainWindow::MainWindow(QWidget *parent) : QMainWindow(parent), ui(new Ui::MainWindowDesign)
{
  ui->setupUi(this);

  QIcon icon("://ros-icon.png");
  this->setWindowIcon(icon);

  qnode = new QNode();
  QObject::connect(qnode, &QNode::rosShutDown, this, &MainWindow::close);

  // ====== 영상 표시 (라벨이 영상 크기로 늘어나 레이아웃이 커지는 것 방지)
  for (QLabel *d : {ui->displayUsb, ui->displayMask, ui->displayBev, ui->displayRoi, ui->displaySkel})
  {
    d->setSizePolicy(QSizePolicy::Ignored, QSizePolicy::Ignored);
    d->setMinimumSize(1, 1);
  }

  connect(qnode, &QNode::usbImageReceived, this, [this](const QImage &img)
          { showImage(ui->displayUsb, img); });
  connect(qnode, &QNode::maskImageReceived, this, [this](const QImage &img)
          { showImage(ui->displayMask, img); });
  connect(qnode, &QNode::bevImageReceived, this, [this](const QImage &img)
          { showImage(ui->displayBev, img); });
  connect(qnode, &QNode::roiImageReceived, this, [this](const QImage &img)
          { showImage(ui->displayRoi, img); });
  connect(qnode, &QNode::skelImageReceived, this, [this](const QImage &img)
          { showImage(ui->displaySkel, img); });

  // ====== HSV 슬라이더 (흰/노랑 대상 전환)
  hsv_widgets_ = {
      {"h_min", {ui->sliderHueLow, ui->dispaly_Hue_Low}},
      {"h_max", {ui->sliderHueHigh, ui->dispaly_Hue_High}},
      {"s_min", {ui->sliderSaturationLow, ui->dispaly_Saturation_Low}},
      {"s_max", {ui->sliderSaturationHigh, ui->dispaly_Saturation_High}},
      {"v_min", {ui->sliderValueLow, ui->dispaly_Value_Low}},
      {"v_max", {ui->sliderValueHigh, ui->dispaly_Value_High}},
  };

  for (auto it = hsv_widgets_.begin(); it != hsv_widgets_.end(); ++it)
  {
    const QString key = it.key();
    QSlider *s = it->slider;
    QLabel *l = it->label;
    s->setRange(0, key.startsWith('h') ? 179 : 255);

    connect(s, &QSlider::valueChanged, this, [this, key, l](int v)
            {
      l->setNum(v);
      const QString param = current_target_ + "." + key;   // 예: "yellow.h_min"
      hsv_cache_[param] = v;
      queueParam(param, v); });
  }

  // ====== BEV 슬라이더
  struct SliderDef
  {
    const char *name;
    QSlider *slider;
    QLabel *label;
  };
  for (const auto &d : {SliderDef{"bev.top_y", ui->sliderBevTopY, ui->dispaly_Bev_Top_Y},
                        SliderDef{"bev.bot_y", ui->sliderBevBotY, ui->dispaly_Bev_Bot_Y},
                        SliderDef{"bev.top_w", ui->sliderBevTopW, ui->dispaly_Bev_Top_W},
                        SliderDef{"bev.bot_w", ui->sliderBevBotW, ui->dispaly_Bev_Bot_W}})
  {
    const QString name = d.name;
    QLabel *l = d.label;
    d.slider->setRange(0, 100);
    global_widgets_[name] = {d.slider, d.label, nullptr};

    connect(d.slider, &QSlider::valueChanged, this, [this, name, l](int v)
            {
      l->setNum(v);
      queueParam(name, v); });
  }

  // ====== 스핀박스 (morph → lineDetect_node, skel → path_node)
  struct SpinDef
  {
    const char *name;
    QSpinBox *spin;
    int min, max;
    bool odd;
  };
  for (const auto &d : {SpinDef{"morph.open", ui->spinMorphOpen, 1, 15, true},
                        SpinDef{"morph.close", ui->spinMorphClose, 1, 31, true},
                        SpinDef{"morph.min_area", ui->spinMinArea, 0, 5000, false},
                        SpinDef{"bev.lane_w_px", ui->spinSkelLaneW, 8, 320, false},
                        SpinDef{"skel.seal_len", ui->spinSkelSealLen, 0, 100, false},
                        SpinDef{"skel.prune_len", ui->spinSkelPruneLen, 0, 100, false},
                        SpinDef{"skel.dash_len", ui->spinSkelDashLen, 0, 160, false}})
  {
    const QString name = d.name;
    QSpinBox *sp = d.spin;
    const bool odd = d.odd;

    sp->setRange(d.min, d.max);
    sp->setSingleStep(odd ? 2 : 1);
    sp->setKeyboardTracking(false);
    global_widgets_[name] = {nullptr, nullptr, sp};

    connect(sp, qOverload<int>(&QSpinBox::valueChanged), this, [this, name, sp, odd](int v)
            {
      if (odd && v % 2 == 0)
      {
        v = std::min(v + 1, sp->maximum());
        QSignalBlocker block(sp);
        sp->setValue(v);
      }
      queueParam(name, v); });
  }

  // 50ms마다 모아서 전송 (드래그 중 요청 폭주 방지)
  send_timer_.setSingleShot(true);
  send_timer_.setInterval(50);
  connect(&send_timer_, &QTimer::timeout, this, [this]
          {
    qnode->setParams(pending_);
    pending_.clear(); });

  // ====== HSV 대상 선택
  ui->radioBtnWhiteLine->setChecked(true);
  connect(ui->radioBtnWhiteLine, &QRadioButton::toggled, this, [this](bool checked)
          {
    if (!checked) return;
    current_target_ = "white";
    loadTargetToSliders(); });
  connect(ui->radioBtnYellowLine, &QRadioButton::toggled, this, [this](bool checked)
          {
    if (!checked) return;
    current_target_ = "yellow";
    loadTargetToSliders(); });

  // ====== 노드에서 읽어온 값 반영
  connect(qnode, &QNode::paramLoaded, this, [this](const QString &name, int v)
          {
    if (pending_.contains(name))
      return;
    if (global_widgets_.contains(name))
    {
      applyGlobalParam(name, v);
      return;
    }
    if (name.startsWith("white.") || name.startsWith("yellow."))
    {
      hsv_cache_[name] = v;
      if (name.startsWith(current_target_ + "."))
        loadTargetToSliders();
    } });

  // 노드가 늦게 떠도 연결되면 현재 값을 한 번 읽어옴
  param_poll_timer_.setInterval(500);
  connect(&param_poll_timer_, &QTimer::timeout, this, [this]
          {
    if (qnode->requestParams())
      param_poll_timer_.stop(); });
  param_poll_timer_.start();

  // ====== 상태 패널
  connect(qnode, &QNode::pathSourceReceived, this, &MainWindow::showPathSource);
  connect(qnode, &QNode::detectionsReceived, this, [this](const QString &t)
          { ui->labelDetections->setText("detections:\n" + t); });

  status_timeout_.setSingleShot(true);
  status_timeout_.setInterval(1000);
  connect(&status_timeout_, &QTimer::timeout, this, [this]
          { showPathSource(""); });

  // ====== 미션 (회전 방향, 주차)
  struct TurnDef
  {
    QRadioButton *btn;
    int turn;
  };
  for (const auto &d : {TurnDef{ui->radioTurnStraight, 0}, TurnDef{ui->radioTurnLeft, 1},
                        TurnDef{ui->radioTurnRight, 2}})
  {
    const int t = d.turn;
    connect(d.btn, &QRadioButton::toggled, this, [this, t](bool checked)
            {
      if (!checked) return;
      turn_ = t;
      qnode->publishTurn(t);
      showMission(); });
  }
  connect(ui->checkParking, &QCheckBox::toggled, this, [this](bool on)
          {
    parking_ = on;
    qnode->publishParking(on);
    showMission(); });

  // 다른 노드가 미션을 바꾸면 버튼 상태만 맞춤 (다시 발행하지 않음)
  connect(qnode, &QNode::turnReceived, this, [this](int t)
          {
    QRadioButton *b = t == 1 ? ui->radioTurnLeft : t == 2 ? ui->radioTurnRight : ui->radioTurnStraight;
    QSignalBlocker block(b);
    b->setChecked(true);
    turn_ = t;
    showMission(); });
  connect(qnode, &QNode::parkingReceived, this, [this](bool on)
          {
    QSignalBlocker block(ui->checkParking);
    ui->checkParking->setChecked(on);
    parking_ = on;
    showMission(); });
  showMission();

  // ====== 파라미터 저장 / 불러오기
  ui->labelParamFile->setText("params: " + paramFile());
  connect(ui->btnSaveParams, &QPushButton::clicked, this, [this]
          { qnode->saveParams(paramFile()); });
  connect(ui->btnLoadParams, &QPushButton::clicked, this, [this]
          { qnode->loadParams(paramFile()); });
  connect(qnode, &QNode::paramFileStatus, ui->labelParamFile, &QLabel::setText);

  // ======== 검출된 cv 객체 표시창 + reset버튼
  connect(qnode, &QNode::signStateChanged, this, [this](const QString &label, int state, const QString &lastTime)
          {
    QLabel *w = nullptr;
    QString name;
    if (label == "parking")
    {
      w = ui->signs_ParkingLabel;
      name = "주차";
    }
    else if (label == "left")
    {
      w = ui->signs_LeftLabel;
      name = "좌회전";
    }
    else if (label == "right")
    {
      w = ui->signs_RightLabel;
      name = "우회전";
    }
    else if (label == "construction")
    {
      w = ui->signs_ConstructionLabel;
      name = "공사장";
    }
    else if (label == "barrier")
    {
      w = ui->signs_BarrierStatusLabel;
      name = "차단바";
    }
    if (!w || state < 0 || state > 2)
      return;

    static const char *mark[] = {"X", "O", "△"};
    static const char *color[] = {"gray", "limegreen", "orange"};
    w->setText(QString("%1 : %2  (%3)").arg(name, mark[state], lastTime));
    w->setStyleSheet(QString("color: %1; font-weight: bold;").arg(color[state])); });

  connect(ui->signsResetButton, &QPushButton::clicked, this, [this]
          { qnode->detectionsReset(); });
}

void MainWindow::showImage(QLabel *label, const QImage &img)
{
  label->setPixmap(QPixmap::fromImage(img).scaled(
      label->size(), Qt::KeepAspectRatio, Qt::FastTransformation));
}

void MainWindow::queueParam(const QString &name, int v)
{
  pending_[name] = v;
  if (!send_timer_.isActive())
    send_timer_.start();
}

void MainWindow::applyGlobalParam(const QString &name, int v)
{
  const GlobalWidget &w = global_widgets_[name];
  if (w.slider && !w.slider->isSliderDown())
  {
    QSignalBlocker block(w.slider);
    w.slider->setValue(v);
    if (w.label)
      w.label->setNum(v);
  }
  if (w.spin && !w.spin->hasFocus())
  {
    QSignalBlocker block(w.spin);
    w.spin->setValue(v);
  }
}

void MainWindow::loadTargetToSliders()
{
  for (auto it = hsv_widgets_.begin(); it != hsv_widgets_.end(); ++it)
  {
    const QString param = current_target_ + "." + it.key();
    if (!hsv_cache_.contains(param) || it->slider->isSliderDown())
      continue;
    const int v = hsv_cache_[param];
    QSignalBlocker block(it->slider);
    it->slider->setValue(v);
    it->label->setNum(v);
  }
}

// live 초록 / memory 보라 / color_rule 주황 / lost 빨강 / 끊김 회색
void MainWindow::showPathSource(const QString &src)
{
  static const QMap<QString, QString> colors = {
      {"live", "#2e9d4a"}, {"memory", "#8e44ad"}, {"color_rule", "#e67e22"}, {"lost", "#c0392b"}};
  const QString bg = colors.value(src, "#555555");
  ui->labelPathSource->setText(src.isEmpty() ? "path: (no data)" : "path: " + src);
  ui->labelPathSource->setStyleSheet(
      QString("font-size: 16px; font-weight: bold; border-radius: 4px; color: white; background: %1;").arg(bg));
  if (!src.isEmpty())
    status_timeout_.start();
}

void MainWindow::showMission()
{
  static const char *names[] = {"straight", "left", "right"};
  ui->labelMission->setText(QString("turn: %1 | parking: %2")
                                .arg(names[std::clamp(turn_, 0, 2)])
                                .arg(parking_ ? "ON" : "off"));
}

QString MainWindow::paramFile() const
{
  return QDir::homePath() + "/tb_params.yaml";
}

void MainWindow::closeEvent(QCloseEvent *event)
{
  QMainWindow::closeEvent(event);
}

MainWindow::~MainWindow()
{
  delete qnode;
  delete ui;
}
