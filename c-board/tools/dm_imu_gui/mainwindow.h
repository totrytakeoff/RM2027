#pragma once

#include <QMainWindow>

class QCheckBox;
class QComboBox;
class QElapsedTimer;
class QGroupBox;
class QLabel;
class QLineEdit;
class QPushButton;
class QSerialPort;
class QTabWidget;
class QTextEdit;
class QTimer;
class QSpinBox;
class QPlainTextEdit;

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = nullptr);
    ~MainWindow() override;

private slots:
    void refreshPorts();
    void connectPort();
    void disconnectPort();
    void onReadyRead();
    void onSendHex();
    void onApplyConfig();
    void onEnterSettings();
    void onExitSettings();
    void onSaveParams();
    void onReboot();
    void onAngleZero();
    void onGyroCal();
    void onAccelCal();
    void onRestoreFactory();
    void onApplySaveExit();
    void updateStats();
    void sendNextCommand();

private:
    void buildUi();
    void logLine(const QString &text);
    void sendCommand(const QByteArray &payload);
    void queueCommand(const QByteArray &payload);
    void queueCommands(const QList<QByteArray> &payloads);
    QByteArray makeCommand(std::initializer_list<uint8_t> bytes);
    void parseBuffer();
    bool parseFrame(const QByteArray &frame);
    void updateDataDisplay();
    void updateRxStats(const QByteArray &chunk);
    void parseVofaStream();
    void applyVofaFrame(const QVector<float> &frame);
    void updateUiState();
    void appendRawRx(const QByteArray &chunk);

    uint16_t crc16(const QByteArray &data) const;
    float readFloatLE(const char *data) const;

    QSerialPort *serial_;
    QByteArray rx_buffer_;

    QComboBox *port_combo_;
    QPushButton *refresh_button_;
    QComboBox *baud_combo_;
    QPushButton *connect_button_;
    QLabel *status_label_;

    QLabel *id_label_;
    QLabel *protocol_label_;
    QLabel *accel_value_[3];
    QLabel *gyro_value_[3];
    QLabel *euler_value_[3];
    QLabel *quat_value_[4];
    QLabel *frame_rate_label_;
    QLabel *crc_error_label_;
    QCheckBox *crc_check_box_;
    QLabel *rx_bytes_label_;
    QLabel *rx_rate_label_;
    QLabel *last_rx_label_;
    QLabel *header_count_label_;
    QCheckBox *vofa_stream_check_;
    QSpinBox *vofa_group_spin_;
    QLabel *vofa_stream_label_;

    QComboBox *iface_combo_;
    QCheckBox *accel_check_;
    QCheckBox *gyro_check_;
    QCheckBox *euler_check_;
    QCheckBox *quat_check_;
    QSpinBox *interval_spin_;
    QSpinBox *can_id_spin_;
    QSpinBox *mst_id_spin_;
    QCheckBox *temp_control_check_;
    QSpinBox *temp_target_spin_;
    QSpinBox *cmd_delay_spin_;

    QLineEdit *hex_input_;
    QPushButton *hex_send_button_;
    QTextEdit *log_view_;
    QPlainTextEdit *raw_view_;
    QPushButton *raw_clear_button_;

    QTimer *stats_timer_;
    QElapsedTimer *fps_timer_;
    QTimer *cmd_timer_;
    QList<QByteArray> cmd_queue_;
    int frame_count_;
    int crc_error_count_;
    qint64 rx_bytes_total_;
    qint64 rx_bytes_window_;
    int header_count_;
    QElapsedTimer *rx_timer_;
    QByteArray last_rx_sample_;
    QVector<float> vofa_cache_;
    QVector<float> vofa_frame_;

    bool is_connected_ = false;
    bool settings_mode_ = false;

    QPushButton *enter_button_;
    QPushButton *exit_button_;
    QPushButton *save_button_;
    QPushButton *apply_save_exit_button_;
    QPushButton *gyro_cal_button_;
    QPushButton *accel_cal_button_;
    QPushButton *angle_zero_button_;
    QPushButton *reboot_button_;
    QPushButton *factory_button_;

    QGroupBox *iface_box_;
    QGroupBox *output_box_;
    QGroupBox *timing_box_;
    QGroupBox *cal_box_;

    struct ImuData
    {
        uint8_t id = 0;
        float accel[3] = {0.0f, 0.0f, 0.0f};
        float gyro[3] = {0.0f, 0.0f, 0.0f};
        float euler[3] = {0.0f, 0.0f, 0.0f};
        float quat[4] = {0.0f, 0.0f, 0.0f, 0.0f};
        bool accel_valid = false;
        bool gyro_valid = false;
        bool euler_valid = false;
        bool quat_valid = false;
    } imu_data_;
};
