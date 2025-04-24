# Warehouse Robotics

## Описание проекта
Проект **Warehouse Robotics** направлен на создание специализированных роботов для автоматизации перемещения и манипуляции ящиками в складских условиях. Решение использует симулятор Gazebo для тестирования алгоритмов и Docker для изоляции среды разработки. Основной стек технологий включает C/C++, ROS (Robot Operating System) и инструменты для работы с робототехникой.

### Основные компоненты
1. **Модели роботов**: Реализация роботов с манипуляторами и системами навигации в Gazebo.
2. **Алгоритмы управления**:
   - Планирование траекторий для захвата ящиков.
   - Навигация в динамической среде.
3. **Интеграция с ROS**: Управление роботами через ROS-ноды и топики.
4. **Docker-контейнеризация**: Изоляция среды выполнения и зависимостей.

## Особенности
- 🚀 Реализация алгоритмов на C/C++ для высокой производительности.
- 📦 Поддержка **определенных** типов ящиков (размеры и вид).
- 🎮 Управление через ROS-интерфейсы.
- 🐳 Docker-образ с предустановленными зависимостями (ROS, Gazebo, Catkin).

## Установка
### Требования
- ОС: Ubuntu 20.04/22.04 (рекомендуется).
- Docker Engine: версия 20.10+.
- ROS: Noetic или Humble (внутри Docker-контейнера).

### Инструкции
1. Клонируйте репозиторий:
   ```bash
   git clone https://github.com/your-username/warehouse-robotics.git
   cd warehouse-robotics
   ```

2. Соберите Docker-образ:
   ```bash
   docker build -t warehouse-robotics:latest -f docker/Dockerfile .
   ```

3. Запустите контейнер:
   ```bash
   docker run -it --rm \
     --env="DISPLAY" \
     --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
     --network=host \
     warehouse-robotics:latest
   ```

## Использование
### Запуск симуляции
1. Внутри контейнера запустите Gazebo с окружением склада:
   ```bash
   roslaunch warehouse_gazebo warehouse.launch
   ```

2. Запустите ноды управления роботом:
   ```bash
   rosrun warehouse_control motion_planner
   ```


## Структура проекта
```
warehouse-robotics/
├── docker/               # Docker-конфигурации
├── src/                  # Исходный код на C/C++
│   ├── control/          # Алгоритмы управления
│   └── models/           # 3D-модели для Gazebo
├── config/               # Параметры ROS и настройки
├── launch/               # ROS-запуски
├── scripts/              # Вспомогательные скрипты
└── README.md
```

## Демо
[Видео работы симуляции](https://google.com).

## Как помочь
1. Форкните репозиторий.
2. Создайте ветку для фичи/исправления:
   ```bash
   git checkout -b feature/your-feature
   ```
3. Откройте Pull Request с описанием изменений.

## Контакты
- Автор: Ярослав Муравьев
- Email: yaroslav.muravev.work@yandex.ru
- Telegram: @yarosalv_muravev

## Лицензия
Проект распространяется под лицензией **Apache License 2.0**. Подробности см. в файле [LICENSE](LICENSE).
