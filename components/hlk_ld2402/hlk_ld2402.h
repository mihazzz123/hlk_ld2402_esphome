#pragma once

#include "esphome/core/component.h"
#include "esphome/core/hal.h"
#include "esphome/components/uart/uart.h"
#include "esphome/components/sensor/sensor.h"
#include "esphome/components/binary_sensor/binary_sensor.h"
#include "esphome/components/text_sensor/text_sensor.h" // Включить без условий

namespace esphome
{
  namespace hlk_ld2402
  {

    static const uint8_t FRAME_HEADER[] = {0xFD, 0xFC, 0xFB, 0xFA};
    static const uint8_t FRAME_FOOTER[] = {0x04, 0x03, 0x02, 0x01};

    // Константы нового формата фрейма
    static const uint8_t DATA_FRAME_HEADER[] = {0xF4, 0xF3, 0xF2, 0xF1}; // Заголовок фрейма данных
    static const uint8_t DATA_FRAME_TYPE_DISTANCE = 0x83;                // Тип фрейма данных расстояния
    static const uint8_t DATA_FRAME_TYPE_ENGINEERING = 0x84;             // Тип фрейма инженерных данныхх

    // Команды
    static const uint16_t CMD_GET_VERSION = 0x0000;              // Команда чтения версии прошивки
    static const uint16_t CMD_ENABLE_CONFIG = 0x00FF;            // Включить режим конфигурации
    static const uint16_t CMD_DISABLE_CONFIG = 0x00FE;           // Завершить режим конфигурации
    static const uint16_t CMD_GET_SN_HEX = 0x0016;               // Прочитать серийный номер (формат hex)
    static const uint16_t CMD_GET_SN_CHAR = 0x0011;              // Прочитать серийный номер (символьный формат)
    static const uint16_t CMD_GET_PARAMS = 0x0008;               // Прочитать параметры
    static const uint16_t CMD_SET_PARAMS = 0x0007;               // Установить параметры
    static const uint16_t CMD_SET_MODE = 0x0012;                 // Установить режим вывода данных
    static const uint16_t CMD_START_CALIBRATION = 0x0009;        // Начать автоматическую генерацию порогов
    static const uint16_t CMD_GET_CALIBRATION_STATUS = 0x000A;   // Запросить прогресс калибровки
    static const uint16_t CMD_CALIBRATION_INTERFERENCE = 0x0014; // Сообщить о помехах при калибровке
    static const uint16_t CMD_SAVE_PARAMS = 0x00FD;              // Сохранить параметры во флеш-память
    static const uint16_t CMD_AUTO_GAIN = 0x00EE;                // Автоматическая регулировка усиления
    static const uint16_t CMD_AUTO_GAIN_COMPLETE = 0x00F0;       // Уведомление о завершении автоусиленияения

    // Параметры
    static const uint16_t PARAM_MAX_DISTANCE = 0x0001;       // Максимальная дистанция обнаружения
    static const uint16_t PARAM_TIMEOUT = 0x0004;            // Задержка исчезновения цели
    static const uint16_t PARAM_POWER_INTERFERENCE = 0x0005; // Статус помех питания (только чтение)
    static const uint16_t PARAM_TRIGGER_THRESHOLD = 0x0010;  // База порога срабатывания движения (0x0010-0x001F)
    static const uint16_t PARAM_MICRO_THRESHOLD = 0x0030;    // База порога микродвижений (0x0030-0x003F)

    // Рабочие режимы
    static const uint32_t MODE_PRODUCTION = 0x00000064; // Обычный производственный режим
    static const uint32_t MODE_NORMAL = 0x00000064;     // Псевдоним для производственного режима
    static const uint32_t MODE_CONFIG = 0x00000001;
    static const uint32_t MODE_ENGINEERING = 0x00000004; // Инженерный/отладочный режим

    // Обновление - Правильная скорость передачи согласно руководствуасно руководству
    static const uint32_t UART_BAUD_RATE = 115200;
    static const uint8_t UART_STOP_BITS = 1;
    static const uint8_t UART_DATA_BITS = 8;
    static const esphome::uart::UARTParityOptions UART_PARITY = esphome::uart::UART_CONFIG_PARITY_NONE;

    // Константы из руководства
    static constexpr float MAX_THEORETICAL_RANGE = 10.0f; // Макс. 10м для движения
    static constexpr float MOVEMENT_RANGE = 10.0f;        // Макс. 10м для движения
    static constexpr float MICROMOVEMENT_RANGE = 6.0f;    // Макс. 6м для микродвижений
    static constexpr float STATIC_RANGE = 5.0f;           // Макс. 5м для статического обнаружения
    static constexpr float DISTANCE_PRECISION = 0.15f;    // Точность ±0.15м
    static constexpr float DISTANCE_GATE_SIZE = 0.7f;     // 0.7м на врата
    static const uint8_t MAX_GATES = 32;                  // Аппаратный максимум врат
    static const uint8_t DEFAULT_GATES = 15;              // Обновлено до 15 для соответствия конфигурации

    // Коэффициенты калибровки
    static const uint8_t DEFAULT_COEFF = 0x1E; // Коэффициент по умолчанию (3.0)
    static const float MIN_COEFF = 1.0f;
    static const float MAX_COEFF = 20.0f;

    class HLKLD2402Component : public Component, public uart::UARTDevice
    {
    public:
      float get_setup_priority() const override { return setup_priority::LATE; }

      void set_distance_sensor(sensor::Sensor *distance_sensor) { distance_sensor_ = distance_sensor; }
      void set_distance_throttle(uint32_t throttle_ms) { distance_throttle_ms_ = throttle_ms; }
      void set_presence_binary_sensor(binary_sensor::BinarySensor *presence) { presence_binary_sensor_ = presence; }
      void set_micromovement_binary_sensor(binary_sensor::BinarySensor *micro) { micromovement_binary_sensor_ = micro; }
      void set_power_interference_binary_sensor(binary_sensor::BinarySensor *power_interference) { power_interference_binary_sensor_ = power_interference; }
      void set_max_distance(float max_distance) { max_distance_ = max_distance; }
      void set_timeout(uint32_t timeout) { timeout_ = timeout; }

      void set_firmware_version_text_sensor(text_sensor::TextSensor *version_sensor)
      {
        this->firmware_version_text_sensor_ = version_sensor;
      }

      void set_operating_mode_text_sensor(text_sensor::TextSensor *mode_sensor)
      {
        this->operating_mode_text_sensor_ = mode_sensor;
      }

      void set_calibration_progress_sensor(sensor::Sensor *calibration_progress) { calibration_progress_sensor_ = calibration_progress; }

      void set_energy_gate_sensor(uint8_t gate_index, sensor::Sensor *energy_sensor)
      {
        if (gate_index < MAX_GATES)
        { // Use the constant for consistency
          if (energy_gate_sensors_.size() <= gate_index)
          {
            energy_gate_sensors_.resize(gate_index + 1, nullptr);
          }
          energy_gate_sensors_[gate_index] = energy_sensor;
          engineering_data_enabled_ = true; // Enable engineering data processing
        }
      }

      // Add threshold sensor setters
      void set_motion_threshold_sensor(uint8_t gate_index, sensor::Sensor *threshold_sensor)
      {
        if (gate_index < MAX_GATES)
        {
          if (motion_threshold_sensors_.size() <= gate_index)
          {
            motion_threshold_sensors_.resize(gate_index + 1, nullptr);
          }
          motion_threshold_sensors_[gate_index] = threshold_sensor;
        }
      }

      void set_micromotion_threshold_sensor(uint8_t gate_index, sensor::Sensor *threshold_sensor)
      {
        if (gate_index < MAX_GATES)
        {
          if (micromotion_threshold_sensors_.size() <= gate_index)
          {
            micromotion_threshold_sensors_.resize(gate_index + 1, nullptr);
          }
          micromotion_threshold_sensors_[gate_index] = threshold_sensor;
        }
      }

      void setup() override;
      void loop() override;
      void dump_config() override;

      void calibrate();
      void save_config();
      void enable_auto_gain();
      void check_power_interference();
      void factory_reset(); // Add new factory reset method

      // Add new direct mode setting methods
      void set_engineering_mode_direct();
      void set_normal_mode_direct();

      // Сохранение существующих методов для обратной совместимости
      void set_engineering_mode();
      void set_normal_mode();

      void get_serial_number();

      // Новые методы установки порогов
      bool set_motion_threshold(uint8_t gate, float db_value);
      bool set_micromotion_threshold(uint8_t gate, float db_value);
      bool calibrate_with_coefficients(float trigger_coeff, float hold_coeff, float micromotion_coeff);

      // Сервис для установки порога движения для конкретных врат
      void set_gate_motion_threshold(int gate, float db_value)
      {
        set_motion_threshold(gate, db_value);
      }

      // Сервис для установки порога микродвижений для конкретных врат
      void set_gate_micromotion_threshold(int gate, float db_value)
      {
        set_micromotion_threshold(gate, db_value);
      }

      // Новые объявления методов для пакетных операций с параметрами
      bool get_all_motion_thresholds();
      bool get_all_micromotion_thresholds();

      // Сервис для чтения порогов
      void read_motion_thresholds()
      {
        get_all_motion_thresholds();
      }

      void read_micromotion_thresholds()
      {
        get_all_micromotion_thresholds();
      }

    protected:
      bool enter_config_mode_();
      bool enter_config_mode_quick_(); // Новый метод быстрого входа
      bool exit_config_mode_();
      bool send_command_(uint16_t command, const uint8_t *data = nullptr, size_t len = 0);
      bool read_response_(std::vector<uint8_t> &response, uint32_t timeout_ms = 1000); // Добавлен параметр таймаута
      bool set_parameter_(uint16_t param_id, uint32_t value);
      bool get_parameter_(uint16_t param_id, uint32_t &value);
      bool set_work_mode_(uint32_t mode);
      bool set_work_mode_with_timeout_(uint32_t mode, uint32_t timeout_ms); // Новый метод с таймаутом
      void process_line_(const std::string &line);
      void dump_hex_(const uint8_t *data, size_t len, const char *prefix);
      bool write_frame_(const std::vector<uint8_t> &frame); // Новый метод
      void get_firmware_version_();                         // Добавление недостающего объявления функции
      void begin_passive_version_detection_();              // Новый метод пассивного обнаружения
      void publish_operating_mode_();                       // Новый метод публикации текущего рабочего режима

      bool save_configuration_();
      bool enable_auto_gain_();
      bool get_serial_number_hex_();
      bool get_serial_number_char_();

      // Преобразование значения дБ в сырой порог
      uint32_t db_to_threshold_(float db_value);
      float threshold_to_db_(uint32_t threshold);

      bool parse_data_frame_(const std::vector<uint8_t> &frame_data);
      bool process_distance_frame_(const std::vector<uint8_t> &frame_data);
      bool process_engineering_data_(const std::vector<uint8_t> &frame_data);
      bool process_engineering_from_distance_frame_(const std::vector<uint8_t> &frame_data); // Новый метод
      void update_binary_sensors_(float distance_cm);                                        // Новый вспомогательный метод

      // Метод пакетного чтения параметров
      bool get_parameters_batch_(const std::vector<uint16_t> &param_ids, std::vector<uint32_t> &values);

    private:
      // Согласно руководству, таймаут ответа должен быть 1с
      static const uint32_t RESPONSE_TIMEOUT_MS = 1000;

      sensor::Sensor *distance_sensor_{nullptr};
      sensor::Sensor *calibration_progress_sensor_{nullptr};
      binary_sensor::BinarySensor *presence_binary_sensor_{nullptr};
      binary_sensor::BinarySensor *micromovement_binary_sensor_{nullptr};
      binary_sensor::BinarySensor *power_interference_binary_sensor_{nullptr};

      text_sensor::TextSensor *firmware_version_text_sensor_{nullptr};
      text_sensor::TextSensor *operating_mode_text_sensor_{nullptr};

      float max_distance_{5.0};
      uint32_t timeout_{5};
      bool config_mode_{false};
      std::string firmware_version_;
      std::string line_buffer_;
      bool power_interference_detected_{false};
      uint32_t last_calibration_status_{0};
      bool calibration_in_progress_{false};
      uint32_t last_calibration_check_{0};                // Time of last calibration check
      uint32_t calibration_progress_{0};                  // Current calibration progress (0-100)
      std::string serial_number_;                         // Добавлено поле для хранения серийного номера
      std::string operating_mode_{"Normal"};              // Отслеживание текущего рабочего режима
      uint32_t last_distance_update_{0};                  // Время последнего обновления датчика расстояния
      uint32_t distance_throttle_ms_{2000};               // Троттлинг по умолчанию 2 секунды
      uint32_t last_engineering_update_{0};               // Время последнего обновления инженерных данных
      uint32_t engineering_throttle_ms_{2000};            // Троттлинг инженерных данных (2 секунды)
      std::vector<sensor::Sensor *> energy_gate_sensors_; // Хранение датчиков врат
      bool engineering_data_enabled_{false};              // Флаг для включения обработки инженерных данных

      // Хранилище для датчиков порогов
      std::vector<sensor::Sensor *> motion_threshold_sensors_;
      std::vector<sensor::Sensor *> micromotion_threshold_sensors_;

      // Кэш для значений порогов
      std::vector<float> motion_threshold_values_;
      std::vector<float> micromotion_threshold_values_;
    };

  } // namespace hlk_ld2402
} // namespace esphome