# METRO 0.4.2 — запуск и проверка

## 1. Образ и панель

Соберите `docker build -t metro-detector:submission .` или загрузите приложенный готовый образ: `docker load -i metro-detector-image.tar.gz`. После загрузки образа интернет не нужен.

Ubuntu 22.04:

```sh
mkdir -p output
docker run --rm -p 127.0.0.1:8190:8190 \
  -v /absolute/path/to/data:/data:ro \
  -v "$PWD/output:/output" \
  metro-detector:submission serve --data /data --host 0.0.0.0 --state /output/state
```

Откройте http://localhost:8190. «Загрузить облако» принимает PCD/PLY/XYZ/CSV. Для bag: «Bag / поток ROS 2» → путь внутри `/data` → «Найти топики» → «Подключить». Загруженные облака и история сохраняются в state. Сохраняйте этот volume между запусками.

Оси по умолчанию: −Y вперёд, +Z вверх; единицы — метры. Для другого монтажа загрузите конфигурацию JSON. Пример всех параметров: config/default.json. Карта и одометрия не требуются для алгоритма.

Windows без Docker: `python -m pip install -r requirements.txt`, затем `start-panel.cmd` и http://localhost:8190. Для другого пути: `python -m metro_detector serve --data D:/your/data --state state`.

## 2. Неизвестный bag, все кадры без пропусков

```sh
docker run --rm -v /absolute/path/to/bag:/data:ro \
  -v "$PWD/output:/output" metro-detector:submission \
  process /data --bag --output /output/detections.jsonl --summary /output/summary.json
```

Если PointCloud2-топиков несколько, добавьте `--topic /actual/topic`. Поддерживаются ROS 2 SQLite3/MCAP. По одной строке JSON на кадр: `obstacle` — тревога, `distance_m` — расстояние от лидара. Обязательно учитывайте `status`; UNCERTAIN не означает свободный путь. Формат подробно описан в docs/OUTPUT.md.

Для просмотра входа: `docker run --rm -v /path/to/bag:/data:ro metro-detector:submission inspect /data`.

## 3. Живой LiDAR или ros2 bag play

Драйвер лидара должен публиковать sensor_msgs/msg/PointCloud2. Сырой UDP декодируется драйвером производителя. Контейнер и источник должны иметь одинаковый ROS_DOMAIN_ID и доступную DDS-сеть. Ubuntu Linux, пример для домена 0:

```sh
docker run --rm --network host -e ROS_DOMAIN_ID=0 \
  --entrypoint /ros_entrypoint.sh metro-detector:submission \
  ros2 launch /app/launch/metro.launch.py topic:=/lidar_points
# В другом терминале ROS_DOMAIN_ID=0:
ros2 bag play /path/to/bag
# Просмотр результата:
ros2 topic echo /metro/obstacles
```

Для панели с живым ROS запускайте `serve --host 0.0.0.0` с `--network host`, затем выберите ROS 2 во вкладке подключения. Docker Desktop Windows не является целевым стендом подключения физического сенсора. Проверка DDS внутри контейнера не заменяет проверку внешней сети и оборудования.

## 4. Самопроверка без датасета

```sh
docker run --rm --network none metro-detector:submission doctor --require-ros
docker run --rm --network none metro-detector:submission \
  process examples/obstacle.xyz --config examples/fixture-config.json --output -
docker run --rm --network none --entrypoint /ros_entrypoint.sh \
  metro-detector:submission python3 /app/tools/ros_smoke.py --launch
```

Примеры проверяют вход/выход, **не качество и не дальность**. fixture-config.json задаёт известный пол синтетического примера; не применяйте его к неизвестным проездам.

## 5. Что посмотреть

- **Обнаружение:** облако и рамки. Shift+клик по точке — XYZ. Красный — тревога; синий — рядом; янтарный — неопределённость. doubleT_obstacle, кадр 24: человек примерно в 56 м. Для режима подтверждения проиграйте несколько последовательных кадров.
- **История обнаружений:** тревожные кадры, фильтры, детали, оценки оператора, экспорт. Оценки не меняют алгоритм. Это кадры, а не уникальные физические объекты.
- **Карта пути:** запустите пример, переключите 2D/3D. Это условный маршрут, реальной локализации по LiDAR нет.
- **Подключение:** инструкции по файлам, bag и потоку.

## Ограничения

Дальность 150+ м и качество на приватных данных не доказаны. На известных отрицательных проездах ложные тревоги проверяются полным прогоном; много результатов остаются неопределёнными. Онлайн-очередь пропускает устаревшие кадры при перегрузке; `process` обрабатывает все кадры. Калибровка и маска своего поезда задаются отдельно. Фактические результаты сдаваемой версии — reports/RELEASE_VALIDATION.md.

## Просмотр обнаружений

На вкладке «Карта пути» красные точки открывают исходное облако с выделенным объектом. Позиции на демонстрационном маршруте условные, события выбраны из журнала (до шести источников). Отмеченные оператором ложные тревоги скрыты на карте и остаются доступны в аналитике. Нажмите «Обновить отметки» после новых обнаружений. В «Истории обнаружений» нажмите «Смотреть в 3D»; доступны приближение к объекту и всё облако. Сохранённый результат не пересчитывается. Старые потоковые записи без облака показывают только границы объекта; новые облака тревог сохраняются в state/previews. Не удаляйте state и исходные файлы, если нужен последующий просмотр.
