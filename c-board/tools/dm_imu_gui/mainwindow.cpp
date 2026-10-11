#include "mainwindow.h"

#include <QCheckBox>
#include <QComboBox>
#include <QElapsedTimer>
#include <QGridLayout>
#include <QGroupBox>
#include <QHBoxLayout>
#include <QLabel>
#include <QLineEdit>
#include <QMainWindow>
#include <QMessageBox>
#include <QPlainTextEdit>
#include <QPushButton>
#include <QSerialPort>
#include <QSerialPortInfo>
#include <QRegularExpression>
#include <QDebug>
#include <QFontDatabase>
#include <QFontMetrics>
#include <QSpinBox>
#include <QTabWidget>
#include <QTextEdit>
#include <QTimer>
#include <QVBoxLayout>

#include <cstring>

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent),
    serial_(new QSerialPort(this)),
    stats_timer_(new QTimer(this)),
    fps_timer_(new QElapsedTimer()),
    cmd_timer_(new QTimer(this)),
    frame_count_(0),
    crc_error_count_(0),
    rx_bytes_total_(0),
    rx_bytes_window_(0),
    header_count_(0),
    rx_timer_(new QElapsedTimer()),
    last_rx_sample_()
{
    buildUi();
    refreshPorts();

    connect(serial_, &QSerialPort::readyRead, this, &MainWindow::onReadyRead);
    connect(stats_timer_, &QTimer::timeout, this, &MainWindow::updateStats);
    connect(cmd_timer_, &QTimer::timeout, this, &MainWindow::sendNextCommand);
    cmd_timer_->setSingleShot(true);
    stats_timer_->start(500);
    fps_timer_->start();
    rx_timer_->start();
}

MainWindow::~MainWindow()
{
    if (serial_->isOpen()) {
        serial_->close();
    }
    delete fps_timer_;
}

void MainWindow::buildUi()
{
    QWidget *central = new QWidget(this);
    QVBoxLayout *root = new QVBoxLayout(central);
    root->setContentsMargins(16, 16, 16, 16);
    root->setSpacing(12);

    QHBoxLayout *top = new QHBoxLayout();
    port_combo_ = new QComboBox(this);
    refresh_button_ = new QPushButton("Refresh", this);
    baud_combo_ = new QComboBox(this);
    baud_combo_->addItems({"115200", "230400", "460800", "921600", "1000000", "2000000"});
    connect_button_ = new QPushButton("Connect", this);
    status_label_ = new QLabel("Disconnected", this);

    top->addWidget(new QLabel("Port:", this));
    top->addWidget(port_combo_);
    top->addWidget(refresh_button_);
    top->addWidget(new QLabel("Baud:", this));
    top->addWidget(baud_combo_);
    top->addWidget(connect_button_);
    top->addWidget(status_label_);
    top->addStretch();

    connect(refresh_button_, &QPushButton::clicked, this, &MainWindow::refreshPorts);
    connect(connect_button_, &QPushButton::clicked, this, &MainWindow::connectPort);

    root->addLayout(top);

    QTabWidget *tabs = new QTabWidget(this);
    root->addWidget(tabs, 1);

    QWidget *data_tab = new QWidget(this);
    QVBoxLayout *data_layout = new QVBoxLayout(data_tab);
    data_layout->setSpacing(12);
    data_layout->setContentsMargins(12, 12, 12, 12);

    id_label_ = new QLabel("ID: --", this);
    protocol_label_ = new QLabel("Protocol: --", this);
    frame_rate_label_ = new QLabel("FPS: 0", this);
    crc_error_label_ = new QLabel("CRC Errors: 0", this);
    crc_check_box_ = new QCheckBox("Enable CRC check", this);
    crc_check_box_->setChecked(true);
    rx_bytes_label_ = new QLabel("RX Bytes: 0", this);
    rx_rate_label_ = new QLabel("RX Rate: 0 B/s", this);
    last_rx_label_ = new QLabel("Last RX: --", this);
    last_rx_label_->setWordWrap(true);
    header_count_label_ = new QLabel("Headers: 0", this);
    vofa_stream_check_ = new QCheckBox("Parse as float stream (VOFA)", this);
    vofa_group_spin_ = new QSpinBox(this);
    vofa_group_spin_->setRange(1, 32);
    vofa_group_spin_->setValue(13);
    vofa_stream_label_ = new QLabel("Stream: --", this);
    vofa_stream_label_->setWordWrap(true);

    auto setMonospace = [this](QLabel *label, const QString &sample) {
        QFont mono = QFontDatabase::systemFont(QFontDatabase::FixedFont);
        label->setFont(mono);
        label->setAlignment(Qt::AlignLeft | Qt::AlignVCenter);
        QFontMetrics metrics(mono);
        label->setMinimumWidth(metrics.horizontalAdvance(sample));
    };
    setMonospace(id_label_, "ID: 0xFF");
    setMonospace(protocol_label_, "Protocol: VOFA float stream");
    setMonospace(frame_rate_label_, "FPS: 0000");
    setMonospace(crc_error_label_, "CRC Errors: 000000");
    setMonospace(rx_bytes_label_, "RX Bytes: 000000000");
    setMonospace(rx_rate_label_, "RX Rate: 000000 B/s");
    setMonospace(vofa_stream_label_, "Stream: f0=+000.0000 f1=+000.0000 f2=+000.0000 f3=+000.0000");
    setMonospace(last_rx_label_, "Last RX: " + QString(64, 'F'));

    QHBoxLayout *info_layout = new QHBoxLayout();
    info_layout->addWidget(id_label_);
    info_layout->addSpacing(12);
    info_layout->addWidget(protocol_label_);
    info_layout->addStretch();
    data_layout->addLayout(info_layout);

    QGroupBox *values_box = new QGroupBox("Live Values", this);
    QGridLayout *values_layout = new QGridLayout(values_box);
    values_layout->setHorizontalSpacing(12);
    values_layout->setVerticalSpacing(6);

    auto makeHeader = [this](const QString &text) {
        QLabel *label = new QLabel(text, this);
        QFont font = label->font();
        font.setBold(true);
        label->setFont(font);
        label->setAlignment(Qt::AlignCenter);
        return label;
    };
    auto makeRowLabel = [this](const QString &text) {
        QLabel *label = new QLabel(text, this);
        QFont font = label->font();
        font.setBold(true);
        label->setFont(font);
        return label;
    };
    auto makeValueLabel = [this, &setMonospace](const QString &sample) {
        QLabel *label = new QLabel(sample, this);
        setMonospace(label, sample);
        label->setAlignment(Qt::AlignRight | Qt::AlignVCenter);
        label->setText("--");
        return label;
    };

    values_layout->addWidget(new QLabel("", this), 0, 0);
    values_layout->addWidget(makeHeader("X"), 0, 1);
    values_layout->addWidget(makeHeader("Y"), 0, 2);
    values_layout->addWidget(makeHeader("Z"), 0, 3);
    values_layout->addWidget(makeHeader("W"), 0, 4);

    values_layout->addWidget(makeRowLabel("Accel"), 1, 0);
    for (int i = 0; i < 3; ++i) {
        accel_value_[i] = makeValueLabel("+000.0000");
        values_layout->addWidget(accel_value_[i], 1, i + 1);
    }
    values_layout->addWidget(new QLabel("--", this), 1, 4);

    values_layout->addWidget(makeRowLabel("Gyro"), 2, 0);
    for (int i = 0; i < 3; ++i) {
        gyro_value_[i] = makeValueLabel("+000.0000");
        values_layout->addWidget(gyro_value_[i], 2, i + 1);
    }
    values_layout->addWidget(new QLabel("--", this), 2, 4);

    values_layout->addWidget(makeRowLabel("Euler"), 3, 0);
    for (int i = 0; i < 3; ++i) {
        euler_value_[i] = makeValueLabel("+000.000");
        values_layout->addWidget(euler_value_[i], 3, i + 1);
    }
    values_layout->addWidget(new QLabel("--", this), 3, 4);

    values_layout->addWidget(makeRowLabel("Quat"), 4, 0);
    for (int i = 0; i < 4; ++i) {
        quat_value_[i] = makeValueLabel("+0.000000");
        values_layout->addWidget(quat_value_[i], 4, i + 1);
    }
    data_layout->addWidget(values_box);

    QGroupBox *stats_box = new QGroupBox("Telemetry", this);
    QGridLayout *stats_layout = new QGridLayout(stats_box);
    stats_layout->addWidget(frame_rate_label_, 0, 0, 1, 1);
    stats_layout->addWidget(crc_error_label_, 0, 1, 1, 1);
    stats_layout->addWidget(rx_bytes_label_, 1, 0, 1, 1);
    stats_layout->addWidget(rx_rate_label_, 1, 1, 1, 1);
    stats_layout->addWidget(header_count_label_, 2, 0, 1, 1);
    stats_layout->addWidget(crc_check_box_, 2, 1, 1, 1);
    data_layout->addWidget(stats_box);

    QGroupBox *stream_box = new QGroupBox("Stream / Raw", this);
    QGridLayout *stream_layout = new QGridLayout(stream_box);
    stream_layout->addWidget(vofa_stream_check_, 0, 0, 1, 1);
    stream_layout->addWidget(new QLabel("Group:", this), 0, 1, 1, 1);
    stream_layout->addWidget(vofa_group_spin_, 0, 2, 1, 1);
    stream_layout->addWidget(vofa_stream_label_, 1, 0, 1, 3);
    stream_layout->addWidget(last_rx_label_, 2, 0, 1, 3);
    data_layout->addWidget(stream_box);
    data_layout->addStretch();

    tabs->addTab(data_tab, "Data");

    QWidget *config_tab = new QWidget(this);
    QVBoxLayout *config_layout = new QVBoxLayout(config_tab);

    iface_box_ = new QGroupBox("Output Interface", this);
    QHBoxLayout *iface_layout = new QHBoxLayout(iface_box_);
    iface_combo_ = new QComboBox(this);
    iface_combo_->addItems({"USB", "RS485", "CAN", "VOFA (JustFloat)"});
    iface_layout->addWidget(new QLabel("Interface:", this));
    iface_layout->addWidget(iface_combo_);
    iface_layout->addStretch();

    output_box_ = new QGroupBox("Output Data", this);
    QGridLayout *output_layout = new QGridLayout(output_box_);
    accel_check_ = new QCheckBox("Accel", this);
    gyro_check_ = new QCheckBox("Gyro", this);
    euler_check_ = new QCheckBox("Euler", this);
    quat_check_ = new QCheckBox("Quaternion", this);
    accel_check_->setChecked(true);
    gyro_check_->setChecked(true);
    euler_check_->setChecked(true);
    quat_check_->setChecked(true);
    output_layout->addWidget(accel_check_, 0, 0);
    output_layout->addWidget(gyro_check_, 0, 1);
    output_layout->addWidget(euler_check_, 1, 0);
    output_layout->addWidget(quat_check_, 1, 1);

    timing_box_ = new QGroupBox("Timing / IDs / Temperature", this);
    QGridLayout *timing_layout = new QGridLayout(timing_box_);
    interval_spin_ = new QSpinBox(this);
    interval_spin_->setRange(1, 1000);
    interval_spin_->setValue(1);
    can_id_spin_ = new QSpinBox(this);
    can_id_spin_->setRange(0, 255);
    can_id_spin_->setValue(1);
    mst_id_spin_ = new QSpinBox(this);
    mst_id_spin_->setRange(0, 255);
    mst_id_spin_->setValue(0x11);
    temp_control_check_ = new QCheckBox("Temp Control", this);
    temp_target_spin_ = new QSpinBox(this);
    temp_target_spin_->setRange(20, 80);
    temp_target_spin_->setValue(40);
    timing_layout->addWidget(new QLabel("Interval (ms):", this), 0, 0);
    timing_layout->addWidget(interval_spin_, 0, 1);
    timing_layout->addWidget(new QLabel("CAN ID:", this), 1, 0);
    timing_layout->addWidget(can_id_spin_, 1, 1);
    timing_layout->addWidget(new QLabel("MST ID:", this), 2, 0);
    timing_layout->addWidget(mst_id_spin_, 2, 1);
    timing_layout->addWidget(temp_control_check_, 3, 0);
    timing_layout->addWidget(new QLabel("Target (C):", this), 3, 1);
    timing_layout->addWidget(temp_target_spin_, 3, 2);
    cmd_delay_spin_ = new QSpinBox(this);
    cmd_delay_spin_->setRange(20, 500);
    cmd_delay_spin_->setValue(120);
    timing_layout->addWidget(new QLabel("Cmd Delay (ms):", this), 4, 0);
    timing_layout->addWidget(cmd_delay_spin_, 4, 1);

    QHBoxLayout *config_buttons = new QHBoxLayout();
    enter_button_ = new QPushButton("Enter Settings", this);
    exit_button_ = new QPushButton("Exit Settings", this);
    save_button_ = new QPushButton("Save Params", this);
    apply_save_exit_button_ = new QPushButton("Apply+Save+Exit", this);
    config_buttons->addWidget(enter_button_);
    config_buttons->addWidget(exit_button_);
    config_buttons->addWidget(save_button_);
    config_buttons->addWidget(apply_save_exit_button_);
    config_buttons->addStretch();

    connect(enter_button_, &QPushButton::clicked, this, &MainWindow::onEnterSettings);
    connect(exit_button_, &QPushButton::clicked, this, &MainWindow::onExitSettings);
    connect(save_button_, &QPushButton::clicked, this, &MainWindow::onSaveParams);
    connect(apply_save_exit_button_, &QPushButton::clicked, this, &MainWindow::onApplySaveExit);

    config_layout->addWidget(iface_box_);
    config_layout->addWidget(output_box_);
    config_layout->addWidget(timing_box_);
    config_layout->addLayout(config_buttons);

    cal_box_ = new QGroupBox("Calibration / Maintenance", this);
    QGridLayout *cal_grid = new QGridLayout(cal_box_);
    gyro_cal_button_ = new QPushButton("Gyro Calibration", this);
    accel_cal_button_ = new QPushButton("Accel Six-Side Cal", this);
    angle_zero_button_ = new QPushButton("Angle Zero", this);
    reboot_button_ = new QPushButton("Reboot IMU", this);
    factory_button_ = new QPushButton("Restore Factory", this);
    cal_grid->addWidget(gyro_cal_button_, 0, 0);
    cal_grid->addWidget(accel_cal_button_, 0, 1);
    cal_grid->addWidget(angle_zero_button_, 1, 0);
    cal_grid->addWidget(reboot_button_, 1, 1);
    cal_grid->addWidget(factory_button_, 2, 0);

    connect(gyro_cal_button_, &QPushButton::clicked, this, &MainWindow::onGyroCal);
    connect(accel_cal_button_, &QPushButton::clicked, this, &MainWindow::onAccelCal);
    connect(angle_zero_button_, &QPushButton::clicked, this, &MainWindow::onAngleZero);
    connect(reboot_button_, &QPushButton::clicked, this, &MainWindow::onReboot);
    connect(factory_button_, &QPushButton::clicked, this, &MainWindow::onRestoreFactory);

    config_layout->addWidget(cal_box_);
    config_layout->addWidget(new QLabel(
        "Note: Calibration requires settings mode and stable placement.",
        this));
    config_layout->addStretch();

    tabs->addTab(config_tab, "Config");

    QWidget *raw_tab = new QWidget(this);
    QVBoxLayout *raw_layout = new QVBoxLayout(raw_tab);
    QHBoxLayout *hex_layout = new QHBoxLayout();
    hex_input_ = new QLineEdit(this);
    hex_send_button_ = new QPushButton("Send Hex", this);
    raw_clear_button_ = new QPushButton("Clear", this);
    hex_layout->addWidget(new QLabel("Hex:", this));
    hex_layout->addWidget(hex_input_);
    hex_layout->addWidget(hex_send_button_);
    hex_layout->addWidget(raw_clear_button_);
    connect(hex_send_button_, &QPushButton::clicked, this, &MainWindow::onSendHex);
    connect(raw_clear_button_, &QPushButton::clicked, [this]() {
        if (raw_view_) {
            raw_view_->clear();
        }
    });

    raw_view_ = new QPlainTextEdit(this);
    raw_view_->setReadOnly(true);
    raw_view_->setMaximumBlockCount(2000);
    raw_layout->addLayout(hex_layout);
    raw_layout->addWidget(raw_view_, 1);
    tabs->addTab(raw_tab, "Raw");

    QWidget *log_tab = new QWidget(this);
    QVBoxLayout *log_layout = new QVBoxLayout(log_tab);
    log_view_ = new QTextEdit(this);
    log_view_->setReadOnly(true);
    log_layout->addWidget(log_view_, 1);
    tabs->addTab(log_tab, "Log");

    setCentralWidget(central);
    setWindowTitle("DM-IMU-L1 Upper Computer");
    resize(900, 600);

    setStyleSheet(
        "QWidget { background: #F6F4F0; color: #1D1D1D; }"
        "QGroupBox { border: 1px solid #D4CDC4; border-radius: 10px; margin-top: 30px; padding: 22px 12px 12px 12px; }"
        "QGroupBox::title { subcontrol-origin: border; subcontrol-position: top left; left: 14px; top: -12px;"
        "background: #EFE6DC; padding: 2px 10px; border: 1px solid #D4CDC4; border-radius: 6px;"
        "font: 700 15px \"Noto Sans\"; color: #1D2F42; }"
        "QPushButton { background: #2E5E87; color: #FFFFFF; border: none; border-radius: 6px; padding: 6px 12px; }"
        "QPushButton:hover { background: #3A74A8; }"
        "QPushButton:disabled { background: #9AA7B1; color: #EEE; }"
        "QLineEdit, QComboBox, QSpinBox, QTextEdit, QPlainTextEdit { background: #FFFFFF; border: 1px solid #D4CDC4; border-radius: 6px; padding: 4px 6px; }"
        "QTabWidget::pane { border: 1px solid #D4CDC4; border-radius: 8px; }"
        "QTabBar::tab { background: #E7E1D8; padding: 6px 10px; border-radius: 6px; margin: 2px; }"
        "QTabBar::tab:selected { background: #FFFFFF; }"
    );

    updateUiState();
}

void MainWindow::refreshPorts()
{
    port_combo_->clear();
    const auto ports = QSerialPortInfo::availablePorts();
    for (const auto &port : ports) {
        port_combo_->addItem(port.portName());
    }
}

void MainWindow::connectPort()
{
    if (serial_->isOpen()) {
        disconnectPort();
        return;
    }

    if (port_combo_->currentText().isEmpty()) {
        QMessageBox::warning(this, "Serial", "No serial port selected.");
        return;
    }

    serial_->setPortName(port_combo_->currentText());
    serial_->setBaudRate(baud_combo_->currentText().toInt());
    serial_->setDataBits(QSerialPort::Data8);
    serial_->setParity(QSerialPort::NoParity);
    serial_->setStopBits(QSerialPort::OneStop);
    serial_->setFlowControl(QSerialPort::NoFlowControl);

    if (!serial_->open(QIODevice::ReadWrite)) {
        QMessageBox::warning(this, "Serial", "Failed to open port.");
        return;
    }

    connect_button_->setText("Disconnect");
    status_label_->setText("Connected");
    is_connected_ = true;
    settings_mode_ = false;
    updateUiState();
    logLine("Serial connected.");
}

void MainWindow::disconnectPort()
{
    if (serial_->isOpen()) {
        serial_->close();
    }
    connect_button_->setText("Connect");
    status_label_->setText("Disconnected");
    is_connected_ = false;
    settings_mode_ = false;
    cmd_queue_.clear();
    cmd_timer_->stop();
    updateUiState();
    logLine("Serial disconnected.");
}

void MainWindow::onReadyRead()
{
    QByteArray chunk = serial_->readAll();
    if (chunk.isEmpty()) {
        return;
    }
    updateRxStats(chunk);
    appendRawRx(chunk);
    rx_buffer_.append(chunk);
    if (vofa_stream_check_->isChecked()) {
        parseVofaStream();
        return;
    }
    parseBuffer();
}

void MainWindow::updateRxStats(const QByteArray &chunk)
{
    rx_bytes_total_ += chunk.size();
    rx_bytes_window_ += chunk.size();
    last_rx_sample_ = chunk;
}

void MainWindow::parseBuffer()
{
    while (true) {
        int idx = rx_buffer_.indexOf(char(0x55));
        if (idx < 0) {
            rx_buffer_.clear();
            break;
        }
        if (idx > 0) {
            rx_buffer_.remove(0, idx);
        }
        if (rx_buffer_.size() < 4) {
            break;
        }
        if (static_cast<uint8_t>(rx_buffer_[1]) != 0xAA) {
            rx_buffer_.remove(0, 1);
            continue;
        }
        header_count_++;
        uint8_t type = static_cast<uint8_t>(rx_buffer_[3]);
        int frame_len = 0;
        if (type >= 1 && type <= 3) {
            frame_len = 19;
        } else if (type == 4) {
            frame_len = 23;
        } else {
            rx_buffer_.remove(0, 1);
            continue;
        }
        if (rx_buffer_.size() < frame_len) {
            break;
        }
        QByteArray frame = rx_buffer_.left(frame_len);
        rx_buffer_.remove(0, frame_len);
        if (parseFrame(frame)) {
            frame_count_++;
        }
    }
}

bool MainWindow::parseFrame(const QByteArray &frame)
{
    if (frame.size() < 6) {
        return false;
    }
    if (static_cast<uint8_t>(frame[0]) != 0x55 || static_cast<uint8_t>(frame[1]) != 0xAA) {
        return false;
    }
    if (static_cast<uint8_t>(frame.back()) != 0x0A) {
        return false;
    }

    if (crc_check_box_->isChecked()) {
        QByteArray crc_data = frame.left(frame.size() - 3);
        uint16_t crc_calc = crc16(crc_data);
        uint16_t crc_le = static_cast<uint8_t>(frame[frame.size() - 3])
                        | (static_cast<uint8_t>(frame[frame.size() - 2]) << 8);
        uint16_t crc_be = (static_cast<uint8_t>(frame[frame.size() - 3]) << 8)
                        | static_cast<uint8_t>(frame[frame.size() - 2]);
        if (crc_calc != crc_le && crc_calc != crc_be) {
            crc_error_count_++;
            crc_error_label_->setText(QString("CRC Errors: %1").arg(crc_error_count_));
            return false;
        }
    }

    imu_data_.id = static_cast<uint8_t>(frame[2]);
    uint8_t type = static_cast<uint8_t>(frame[3]);
    const char *payload = frame.constData() + 4;

    if (type == 1) {
        for (int i = 0; i < 3; ++i) {
            imu_data_.accel[i] = readFloatLE(payload + i * 4);
        }
        imu_data_.accel_valid = true;
    } else if (type == 2) {
        for (int i = 0; i < 3; ++i) {
            imu_data_.gyro[i] = readFloatLE(payload + i * 4);
        }
        imu_data_.gyro_valid = true;
    } else if (type == 3) {
        for (int i = 0; i < 3; ++i) {
            imu_data_.euler[i] = readFloatLE(payload + i * 4);
        }
        imu_data_.euler_valid = true;
    } else if (type == 4) {
        for (int i = 0; i < 4; ++i) {
            imu_data_.quat[i] = readFloatLE(payload + i * 4);
        }
        imu_data_.quat_valid = true;
    } else {
        return false;
    }

    updateDataDisplay();
    return true;
}

void MainWindow::parseVofaStream()
{
    while (rx_buffer_.size() >= 4) {
        uint32_t raw = 0;
        std::memcpy(&raw, rx_buffer_.constData(), sizeof(uint32_t));
        float value = readFloatLE(rx_buffer_.constData());
        rx_buffer_.remove(0, 4);
        if (raw == 0x7F800000u) {
            if (!vofa_frame_.isEmpty()) {
                applyVofaFrame(vofa_frame_);
                vofa_frame_.clear();
            }
            continue;
        }
        vofa_frame_.push_back(value);
    }
}

void MainWindow::applyVofaFrame(const QVector<float> &frame)
{
    int group = vofa_group_spin_->value();
    if (group <= 0) {
        return;
    }
    if (frame.size() < group) {
        return;
    }

    QVector<float> latest;
    latest.reserve(group);
    for (int i = frame.size() - group; i < frame.size(); ++i) {
        latest.push_back(frame[i]);
    }

    QStringList parts;
    for (int i = 0; i < latest.size(); ++i) {
        parts << QString("f%1=%2").arg(i).arg(latest[i], 0, 'f', 4);
    }
    vofa_stream_label_->setText("Stream: " + parts.join(" "));

    if (group == 13) {
        for (int i = 0; i < 3; ++i) {
            imu_data_.accel[i] = latest[i];
            imu_data_.gyro[i] = latest[i + 3];
            imu_data_.euler[i] = latest[i + 6];
        }
        for (int i = 0; i < 4; ++i) {
            imu_data_.quat[i] = latest[i + 9];
        }
        imu_data_.accel_valid = true;
        imu_data_.gyro_valid = true;
        imu_data_.euler_valid = true;
        imu_data_.quat_valid = true;
        updateDataDisplay();
    }
}

void MainWindow::updateDataDisplay()
{
    id_label_->setText(QString("ID: 0x%1").arg(imu_data_.id, 2, 16, QLatin1Char('0')));
    if (imu_data_.accel_valid) {
        for (int i = 0; i < 3; ++i) {
            accel_value_[i]->setText(QString("%1").arg(imu_data_.accel[i], 0, 'f', 4));
        }
    }
    if (imu_data_.gyro_valid) {
        for (int i = 0; i < 3; ++i) {
            gyro_value_[i]->setText(QString("%1").arg(imu_data_.gyro[i], 0, 'f', 4));
        }
    }
    if (imu_data_.euler_valid) {
        for (int i = 0; i < 3; ++i) {
            euler_value_[i]->setText(QString("%1").arg(imu_data_.euler[i], 0, 'f', 3));
        }
    }
    if (imu_data_.quat_valid) {
        for (int i = 0; i < 4; ++i) {
            quat_value_[i]->setText(QString("%1").arg(imu_data_.quat[i], 0, 'f', 6));
        }
    }
}

void MainWindow::onSendHex()
{
    const QString text = hex_input_->text().trimmed();
    if (text.isEmpty()) {
        return;
    }
    QStringList tokens = text.split(QRegularExpression("\\s+"), Qt::SkipEmptyParts);
    QByteArray payload;
    for (const QString &token : tokens) {
        bool ok = false;
        int value = token.toInt(&ok, 16);
        if (!ok || value < 0 || value > 255) {
            QMessageBox::warning(this, "Hex", "Invalid hex token: " + token);
            return;
        }
        payload.append(static_cast<char>(value));
    }
    sendCommand(payload);
    logLine("TX: " + text.toUpper());
}

void MainWindow::onApplyConfig()
{
    int iface = iface_combo_->currentIndex();
    QList<QByteArray> cmds;
    cmds.push_back(makeCommand({0xAA, 0x0A, static_cast<uint8_t>(iface), 0x0D}));
    cmds.push_back(makeCommand({0xAA, 0x01,
        static_cast<uint8_t>(accel_check_->isChecked() ? 0x14 : 0x04), 0x0D}));
    cmds.push_back(makeCommand({0xAA, 0x01,
        static_cast<uint8_t>(gyro_check_->isChecked() ? 0x15 : 0x05), 0x0D}));
    cmds.push_back(makeCommand({0xAA, 0x01,
        static_cast<uint8_t>(euler_check_->isChecked() ? 0x16 : 0x06), 0x0D}));
    cmds.push_back(makeCommand({0xAA, 0x01,
        static_cast<uint8_t>(quat_check_->isChecked() ? 0x17 : 0x07), 0x0D}));
    uint16_t interval = static_cast<uint16_t>(interval_spin_->value());
    uint8_t low = static_cast<uint8_t>(interval & 0xFF);
    uint8_t high = static_cast<uint8_t>((interval >> 8) & 0xFF);
    cmds.push_back(makeCommand({0xAA, 0x02, low, high, 0x0D}));
    cmds.push_back(makeCommand({0xAA, 0x08, static_cast<uint8_t>(can_id_spin_->value()), 0x0D}));
    cmds.push_back(makeCommand({0xAA, 0x09, static_cast<uint8_t>(mst_id_spin_->value()), 0x0D}));
    cmds.push_back(makeCommand({0xAA, 0x04, static_cast<uint8_t>(temp_control_check_->isChecked() ? 1 : 0), 0x0D}));
    cmds.push_back(makeCommand({0xAA, 0x05, static_cast<uint8_t>(temp_target_spin_->value()), 0x0D}));
    queueCommands(cmds);
    logLine("Applied configuration commands.");
}

void MainWindow::onEnterSettings()
{
    queueCommand(makeCommand({0xAA, 0x06, 0x01, 0x0D}));
    logLine("Enter settings mode.");
}

void MainWindow::onExitSettings()
{
    queueCommand(makeCommand({0xAA, 0x06, 0x00, 0x0D}));
    logLine("Exit settings mode.");
}

void MainWindow::onSaveParams()
{
    queueCommand(makeCommand({0xAA, 0x03, 0x01, 0x0D}));
    logLine("Save parameters.");
}

void MainWindow::onApplySaveExit()
{
    queueCommand(makeCommand({0xAA, 0x06, 0x01, 0x0D}));
    onApplyConfig();
    queueCommand(makeCommand({0xAA, 0x03, 0x01, 0x0D}));
    queueCommand(makeCommand({0xAA, 0x06, 0x00, 0x0D}));
    logLine("Applied, saved, and exited settings mode.");
}

void MainWindow::onReboot()
{
    queueCommand(makeCommand({0xAA, 0x00, 0x00, 0x0D}));
    logLine("Reboot IMU.");
}

void MainWindow::onAngleZero()
{
    queueCommand(makeCommand({0xAA, 0x0C, 0x01, 0x0D}));
    logLine("Angle zero.");
}

void MainWindow::onGyroCal()
{
    queueCommand(makeCommand({0xAA, 0x03, 0x02, 0x0D}));
    logLine("Gyro calibration.");
}

void MainWindow::onAccelCal()
{
    queueCommand(makeCommand({0xAA, 0x03, 0x03, 0x0D}));
    logLine("Accel six-side calibration.");
}

void MainWindow::onRestoreFactory()
{
    queueCommand(makeCommand({0xAA, 0x0B, 0x01, 0x0D}));
    logLine("Restore factory settings.");
}

void MainWindow::updateStats()
{
    double seconds = fps_timer_->elapsed() / 1000.0;
    if (seconds <= 0.0) {
        return;
    }
    int fps = static_cast<int>(frame_count_ / seconds);
    frame_rate_label_->setText(QString("FPS: %1").arg(fps));
    if (seconds > 2.0) {
        frame_count_ = 0;
        fps_timer_->restart();
    }

    double rx_seconds = rx_timer_->elapsed() / 1000.0;
    if (rx_seconds >= 1.0) {
        double rate = rx_bytes_window_ / rx_seconds;
        rx_rate_label_->setText(QString("RX Rate: %1 B/s").arg(static_cast<int>(rate)));
        rx_bytes_label_->setText(QString("RX Bytes: %1").arg(rx_bytes_total_));
        header_count_label_->setText(QString("Headers: %1").arg(header_count_));
        rx_bytes_window_ = 0;
        rx_timer_->restart();

        qDebug() << "RX bytes" << rx_bytes_total_
                 << "rate" << static_cast<int>(rate)
                 << "headers" << header_count_
                 << "crcErrors" << crc_error_count_
                 << "vofa" << vofa_stream_check_->isChecked();
    }

    if (!last_rx_sample_.isEmpty()) {
        QByteArray preview = last_rx_sample_.left(32).toHex(' ').toUpper();
        last_rx_label_->setText("Last RX: " + QString(preview));
        qDebug() << "Last RX:" << QString(preview);
    } else {
        last_rx_label_->setText("Last RX: --");
    }

    if (vofa_stream_check_->isChecked()) {
        protocol_label_->setText("Protocol: VOFA float frame");
    } else if (header_count_ > 0) {
        protocol_label_->setText("Protocol: USB frame");
    } else {
        protocol_label_->setText("Protocol: Unknown");
    }
}

void MainWindow::logLine(const QString &text)
{
    log_view_->append(text);
}

void MainWindow::appendRawRx(const QByteArray &chunk)
{
    if (!raw_view_) {
        return;
    }
    const QString hex = chunk.toHex(' ').toUpper();
    if (!hex.isEmpty()) {
        raw_view_->appendPlainText(hex);
    }
}

void MainWindow::updateUiState()
{
    enter_button_->setEnabled(is_connected_ && !settings_mode_);
    exit_button_->setEnabled(is_connected_ && settings_mode_);
    save_button_->setEnabled(is_connected_ && settings_mode_);
    apply_save_exit_button_->setEnabled(is_connected_ && settings_mode_);

    iface_box_->setEnabled(is_connected_ && settings_mode_);
    output_box_->setEnabled(is_connected_ && settings_mode_);
    timing_box_->setEnabled(is_connected_ && settings_mode_);
    cal_box_->setEnabled(is_connected_ && settings_mode_);

    hex_send_button_->setEnabled(is_connected_);
    raw_clear_button_->setEnabled(is_connected_);
}

void MainWindow::sendCommand(const QByteArray &payload)
{
    if (!serial_->isOpen()) {
        QMessageBox::warning(this, "Serial", "Port not connected.");
        return;
    }
    serial_->write(payload);
}

void MainWindow::queueCommand(const QByteArray &payload)
{
    cmd_queue_.push_back(payload);
    if (!cmd_timer_->isActive()) {
        sendNextCommand();
    }
}

void MainWindow::queueCommands(const QList<QByteArray> &payloads)
{
    for (const auto &payload : payloads) {
        cmd_queue_.push_back(payload);
    }
    if (!cmd_timer_->isActive()) {
        sendNextCommand();
    }
}

void MainWindow::sendNextCommand()
{
    if (cmd_queue_.isEmpty()) {
        cmd_timer_->stop();
        return;
    }
    QByteArray payload = cmd_queue_.takeFirst();
    sendCommand(payload);
    if (payload.size() >= 4 && static_cast<uint8_t>(payload[0]) == 0xAA
        && static_cast<uint8_t>(payload.back()) == 0x0D) {
        const uint8_t cmd = static_cast<uint8_t>(payload[1]);
        const uint8_t arg = static_cast<uint8_t>(payload[2]);
        if (cmd == 0x06) {
            settings_mode_ = (arg == 0x01);
        } else if (cmd == 0x00) {
            settings_mode_ = false;
        } else if (cmd == 0x03 && (arg == 0x02 || arg == 0x03)) {
            settings_mode_ = false;
        } else if (cmd == 0x0B) {
            settings_mode_ = false;
        }
        updateUiState();
    }
    cmd_timer_->start(cmd_delay_spin_->value());
}

QByteArray MainWindow::makeCommand(std::initializer_list<uint8_t> bytes)
{
    QByteArray payload;
    payload.reserve(static_cast<int>(bytes.size()));
    for (uint8_t b : bytes) {
        payload.append(static_cast<char>(b));
    }
    return payload;
}

uint16_t MainWindow::crc16(const QByteArray &data) const
{
    static const uint16_t table[256] = {
        0x0000, 0x1021, 0x2042, 0x3063, 0x4084, 0x50A5, 0x60C6, 0x70E7, 0x8108, 0x9129,
        0xA14A, 0xB16B, 0xC18C, 0xD1AD, 0xE1CE, 0xF1EF, 0x1231, 0x0210, 0x3273, 0x2252,
        0x52B5, 0x4294, 0x72F7, 0x62D6, 0x9339, 0x8318, 0xB37B, 0xA35A, 0xD3BD, 0xC39C,
        0xF3FF, 0xE3DE, 0x2462, 0x3443, 0x0420, 0x1401, 0x64E6, 0x74C7, 0x44A4, 0x5485,
        0xA56A, 0xB54B, 0x8528, 0x9509, 0xE5EE, 0xF5CF, 0xC5AC, 0xD58D, 0x3653, 0x2672,
        0x1611, 0x0630, 0x76D7, 0x66F6, 0x5695, 0x46B4, 0xB75B, 0xA77A, 0x9719, 0x8738,
        0xF7DF, 0xE7FE, 0xD79D, 0xC7BC, 0x48C4, 0x58E5, 0x6886, 0x78A7, 0x0840, 0x1861,
        0x2802, 0x3823, 0xC9CC, 0xD9ED, 0xE98E, 0xF9AF, 0x8948, 0x9969, 0xA90A, 0xB92B,
        0x5AF5, 0x4AD4, 0x7AB7, 0x6A96, 0x1A71, 0x0A50, 0x3A33, 0x2A12, 0xDBFD, 0xCBDC,
        0xFBBF, 0xEB9E, 0x9B79, 0x8B58, 0xBB3B, 0xAB1A, 0x6CA6, 0x7C87, 0x4CE4, 0x5CC5,
        0x2C22, 0x3C03, 0x0C60, 0x1C41, 0xEDAE, 0xFD8F, 0xCDEC, 0xDDCD, 0xAD2A, 0xBD0B,
        0x8D68, 0x9D49, 0x7E97, 0x6EB6, 0x5ED5, 0x4EF4, 0x3E13, 0x2E32, 0x1E51, 0x0E70,
        0xFF9F, 0xEFBE, 0xDFDD, 0xCFFC, 0xBF1B, 0xAF3A, 0x9F59, 0x8F78, 0x9188, 0x81A9,
        0xB1CA, 0xA1EB, 0xD10C, 0xC12D, 0xF14E, 0xE16F, 0x1080, 0x00A1, 0x30C2, 0x20E3,
        0x5004, 0x4025, 0x7046, 0x6067, 0x83B9, 0x9398, 0xA3FB, 0xB3DA, 0xC33D, 0xD31C,
        0xE37F, 0xF35E, 0x02B1, 0x1290, 0x22F3, 0x32D2, 0x4235, 0x5214, 0x6277, 0x7256,
        0xB5EA, 0xA5CB, 0x95A8, 0x8589, 0xF56E, 0xE54F, 0xD52C, 0xC50D, 0x34E2, 0x24C3,
        0x14A0, 0x0481, 0x7466, 0x6447, 0x5424, 0x4405, 0xA7DB, 0xB7FA, 0x8799, 0x97B8,
        0xE75F, 0xF77E, 0xC71D, 0xD73C, 0x26D3, 0x36F2, 0x0691, 0x16B0, 0x6657, 0x7676,
        0x4615, 0x5634, 0xD94C, 0xC96D, 0xF90E, 0xE92F, 0x99C8, 0x89E9, 0xB98A, 0xA9AB,
        0x5844, 0x4865, 0x7806, 0x6827, 0x18C0, 0x08E1, 0x3882, 0x28A3, 0xCB7D, 0xDB5C,
        0xEB3F, 0xFB1E, 0x8BF9, 0x9BD8, 0xABBB, 0xBB9A, 0x4A75, 0x5A54, 0x6A37, 0x7A16,
        0x0AF1, 0x1AD0, 0x2AB3, 0x3A92, 0xFD2E, 0xED0F, 0xDD6C, 0xCD4D, 0xBDAA, 0xAD8B,
        0x9DE8, 0x8DC9, 0x7C26, 0x6C07, 0x5C64, 0x4C45, 0x3CA2, 0x2C83, 0x1CE0, 0x0CC1,
        0xEF1F, 0xFF3E, 0xCF5D, 0xDF7C, 0xAF9B, 0xBFBA, 0x8FD9, 0x9FF8, 0x6E17, 0x7E36,
        0x4E55, 0x5E74, 0x2E93, 0x3EB2, 0x0ED1, 0x1EF0
    };

    uint16_t crc = 0xFFFF;
    for (uint8_t b : data) {
        uint8_t index = static_cast<uint8_t>((crc >> 8) ^ b);
        crc = static_cast<uint16_t>(((crc << 1) & 0xFFFF) ^ table[index]);
    }
    return crc;
}

float MainWindow::readFloatLE(const char *data) const
{
    float value = 0.0f;
    std::memcpy(&value, data, sizeof(float));
    return value;
}
