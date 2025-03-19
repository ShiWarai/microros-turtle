# Образ Docker для работы с роботом-черепахой

Добро пожаловать в руководство по настройке и использованию Docker-образа для работы с роботом-черепахой на базе ROS 2 Humble и microROS. Этот образ позволяет разрабатывать под microROS и ROS 2 на системах без поддержки последних, но с поддержкой Docker. В текущем варианте работа протестирована на операционной системе Windows 11 (но Windows 10 тоже подойдёт).

---

## Оглавление

1. [Требования](#требования)
2. [Установка зависимостей](#установка-зависимостей)
3. [Сборка образа](#сборка-образа)
4. [Запуск контейнера](#запуск-контейнера)
5. [Настройка X-Server для GUI](#настройка-x-server-для-gui)
6. [Работа с microROS](#работа-с-microROS)
7. [Пример кода для камеры](#пример-кода-для-камеры)
8. [Разработка внутри контейнера](#разработка-внутри-контейнера)
9. [Полезные советы](#полезные-советы)

---

## Требования

Для работы с образом вам понадобится:
- **Операционная система**: Windows 10 и выше
- **Docker Desktop**: Установлен и запущен.
- **VS Code**: Рекомендуется для разработки внутри контейнера.
- **X-Server**: Например, VcXsrv для поддержки GUI в Windows.
- Базовые знания ROS 2 и Python.

---

## Установка зависимостей

Перед началом убедитесь, что все необходимые инструменты установлены:

1. **Docker Desktop**:
   - Скачайте с [официального сайта](https://www.docker.com/products/docker-desktop/).
   - Установите и запустите. Проверьте работоспособность:
     ```bash
     docker --version
     ```

2. **VcXsrv (X-Server)**:
   - Скачайте с [SourceForge](https://sourceforge.net/projects/vcxsrv/).
   - Установите и настройте (см. раздел [Настройка X-Server](#настройка-x-server-для-gui)).

3. **VS Code и расширения** (опционально, для разработки):
   - Установите [VS Code](https://code.visualstudio.com/).
   - Установите расширения:
     - `Docker` (Microsoft).
     - `Dev Containers` (Microsoft).

4. **Python и ROS 2 на хосте** (опционально, для проверки кода):
   - Установите Python 3.9+.
   - Убедитесь, что OpenCV (`cv2`) доступен:
     ```bash
     pip install opencv-python
     ```

---

## Сборка образа

Для создания Docker-образа выполните следующие шаги:

1. Склонируйте репозиторий или создайте папку с `Dockerfile`.
2. Убедитесь, что в папке есть `Dockerfile` и, при необходимости, папка `extra_packages`.
3. Откройте терминал в этой папке и выполните:
   ```bash
   docker build -t microros-humble-custom .

- Замените `microros-humble-custom` на желаемое имя образа.

> **Примечание**: Процесс сборки может занять несколько минут в зависимости от скорости интернета и мощности ПК.

---

## Запуск контейнера

После сборки запустите контейнер с помощью скрипта:

1. Убедитесь, что файл `run_container_on_windows.bat` находится в той же директории.
2. Выполните:
   ```bash
   .\run_container_on_windows.bat microros-humble-custom
- Если имя образа отличается, укажите его как аргумент.

Скрипт:
- Определит IP-адрес хоста.
- Создаст папку `ros2_ws` в текущей директории (если её нет).
- Смонтирует `ros2_ws` в `/ros2_ws` внутри контейнера.
- Прокинет порт `8888` для microROS.

---

## Настройка X-Server для GUI

Для работы с GUI-приложениями (например, `cv2.imshow`) настройте X-Server:

1. Запустите **VcXsrv**:
   - Откройте `XLaunch` из меню "Пуск".
   - Используйте файл `config.xlaunch` из репозитория или создайте новый:
     - Выберите `Multiple windows`.
     - Укажите `Display number: 0`.
     - Включите `Disable access control` (отметьте галочку).
     - Сохраните конфигурацию и запустите.

2. Проверьте переменную `DISPLAY`:
   - Скрипт автоматически устанавливает `DISPLAY=host.docker.internal:0`. Если это не работает, замените в скрипте на ваш IP-адрес (например, `192.168.1.100:0`).

---

## Работа с microROS

Для запуска microROS агента внутри контейнера:

1. В терминале контейнера выполните:
   ```bash
   ros2 run micro_ros_agent micro_ros_agent udp4 --port 8888 -v4
2. Или добавьте эту команду в ваш ROS 2 launch-файл.

> **Совет**: Не используйте `ENTRYPOINT` для агента в `Dockerfile`, чтобы сохранить возможность его перезапуска.

---

## Пример кода для камеры

Этот код демонстрирует получение изображений с камеры робота через ROS 2 и их отображение с помощью OpenCV:

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
import cv2
import numpy as np

class ImageViewer(Node):
    def __init__(self):
        super().__init__('image_viewer')
        self.subscription = self.create_subscription(
            CompressedImage,
            '/camera/image_raw',
            self.image_callback,
            10
        )
        self.get_logger().info('Image Viewer Node has started')

    def image_callback(self, msg):
        # Преобразование сжатого изображения в массив NumPy
        np_arr = np.frombuffer(msg.data, np.uint8)
        image_np = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

        if image_np is not None:
            # Поворот изображения на 90 градусов по часовой стрелке (опционально)
            # image_np = cv2.rotate(image_np, cv2.ROTATE_90_CLOCKWISE)
            
            cv2.imshow("ESP32-CAM Image", image_np)
            cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    image_viewer = ImageViewer()

    try:
        rclpy.spin(image_viewer)
    except KeyboardInterrupt:
        print("Прерывание по Ctrl+C")
    
    image_viewer.destroy_node()
    rclpy.shutdown()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
```

**Особенности**:
- Используется `CompressedImage` для экономии пропускной способности.
- Убедитесь, что X-Server настроен, иначе `cv2.imshow` не сработает.

---

## Разработка внутри контейнера

Для написания пакетов и работы с microROS используйте VS Code с Dev Containers:

1. Установите расширения `Docker` и `Dev Containers` в VS Code.
2. Откройте папку проекта в VS Code.
3. Нажмите `Ctrl+Shift+P` > **Dev Containers: Reopen in Container**.
4. Подробности в [документации Microsoft](https://code.visualstudio.com/docs/devcontainers/containers).

> **Преимущества**: Все зависимости уже настроены в контейнере, включая ROS 2 и microROS.

---

## Полезные советы

- **Монтирование**: Папка `ros2_ws` на хосте синхронизируется с `/ros2_ws` в контейнере. Добавляйте свои пакеты туда.
- **Перезапуск агента**: Используйте отдельный терминал для `micro_ros_agent`, чтобы перезапускать его без остановки контейнера.
- **Логи**: Добавьте `-v6` к команде агента для подробных логов.
- **Проблемы с GUI**: Если изображение не отображается, проверьте IP в `DISPLAY` и настройки X-Server.