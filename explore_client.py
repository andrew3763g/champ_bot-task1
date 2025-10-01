#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
========================================
Робот-навигатор для лабиринта из двух комнат
========================================

Описание:
    Программа управляет роботом-машинкой в лабиринте из двух комнат.
    Робот начинает в "кармане" из утят, должен выехать назад, повернуть на восток,
    проехать через первую комнату, пройти коридор, вторую комнату и остановиться
    на белом прямоугольнике (финише).

Стратегия:
    - Приоритет движения на ВОСТОК (направление определяется автоматически)
    - Адаптивный объезд препятствий на основе данных лидара
    - Конечный автомат с чёткими фазами движения
    - Детальное логирование в explore.log

Автор: Claude Code
Дата: 2025
"""

# ==================== Импорт библиотек ====================
import math    # Математические функции (sin, cos, pi и т.д.)
import os      # Работа с операционной системой (переменные окружения)
import socket  # Сетевое взаимодействие (UDP/TCP)
import struct  # Упаковка/распаковка бинарных данных
import time    # Работа со временем (таймауты, задержки)
import sys     # Для перенаправления вывода в лог
from enum import Enum  # Для создания перечислений (состояний робота)

# ==================== Настройка логирования ====================
# Перенаправляем print в файл explore.log
original_stdout = sys.stdout  # Сохраняем оригинальный stdout
log_file = open("explore.log", "w", encoding="utf-8", buffering=1)  # buffering=1 = построчная буферизация

# Класс для дублирования вывода в консоль и файл одновременно
class TeeOutput:
    def __init__(self, *files):
        self.files = files

    def write(self, data):
        for f in self.files:
            f.write(data)
            f.flush()

    def flush(self):
        for f in self.files:
            f.flush()

# Перенаправляем stdout в консоль И файл одновременно
sys.stdout = TeeOutput(original_stdout, log_file)


# ==================== Конфигурация ====================

# --- Параметры сети ---
# Эти параметры определяют, куда робот отправляет команды и откуда получает данные
CMD_HOST = os.getenv("CMD_HOST", "127.0.0.1")  # IP-адрес для отправки команд (по умолчанию localhost)
CMD_PORT = int(os.getenv("CMD_PORT", "5555"))   # Порт для отправки команд (UDP)
TEL_HOST = os.getenv("TEL_HOST", "0.0.0.0")     # IP для приёма телеметрии (0.0.0.0 = все интерфейсы)
TEL_PORT = int(os.getenv("TEL_PORT", "5600"))   # Порт для приёма телеметрии
PROTO = os.getenv("PROTO", "tcp")               # Протокол телеметрии: "tcp" или "udp"

# --- Параметры лидара ---
# Лидар - это датчик, который измеряет расстояние до препятствий вокруг робота
LIDAR_HALF_FOV = math.pi / 4  # Половина угла обзора лидара: ±45° (всего 90°)
LIDAR_FOV = 2 * LIDAR_HALF_FOV  # Полный угол обзора лидара: 90° = π/2 радиан
MAX_RANGE = 6.0               # Максимальная дальность лидара в метрах

# --- Безопасные дистанции ---
# Эти параметры определяют, насколько близко робот может подъехать к препятствию
OBSTACLE_DIST = 0.45  # Дистанция, при которой препятствие считается "близким" (метры)
SAFE_DIST = 0.60      # Безопасная дистанция для комфортного движения
WALL_FOLLOW_DIST = 0.50  # Желаемая дистанция до стены при движении вдоль неё

# --- Скорости движения ---
# v (linear velocity) - линейная скорость вперёд/назад (м/с)
# w (angular velocity) - угловая скорость поворота (рад/с)
V_FAST = 0.70    # Быстрое движение вперёд (когда путь свободен)
V_NORMAL = 0.50  # Нормальная скорость (когда есть небольшие препятствия)
V_SLOW = 0.25    # Медленная скорость (при объезде, поворотах)
W_MAX = 1.5      # Максимальная угловая скорость (скорость поворота)

# --- Параметры фаз движения ---
EXIT_POCKET_BACKUP = True   # Ехать назад до упора в стену (вместо фиксированной дистанции)
EXIT_POCKET_DIST = 1.8      # Минимальное расстояние выезда из кармана (если не уперлись в стену) - УВЕЛИЧЕНО!
CORRIDOR_WIDTH_MAX = 1.0    # Максимальная ширина по бокам для детекции коридора
ROOM_WIDTH_MIN = 1.5        # Минимальная ширина по бокам для детекции комнаты
FINISH_ZONE_OPEN = 2.5      # Минимальное открытое пространство для детекции финиша
FINISH_RADIUS = 0.3         # Радиус (метры), в котором считается, что робот на финише

# --- Таймауты ---
TIMEOUT_STUCK = 3.0   # Сколько секунд без движения = "робот застрял"
TIMEOUT_MAX = 200.0   # Максимальное время работы программы (секунды)

# --- Калибровка направлений ---
# Эти значения будут установлены автоматически при первом получении телеметрии
NORTH_ANGLE = None   # Угол "север" (начальная ориентация робота в кармане)
EAST_ANGLE = None    # Угол "восток" (будет вычислен как NORTH_ANGLE - π/2 или + π/2)


# ==================== Состояния робота ====================
# Enum - это перечисление, удобный способ задать набор состояний
# Робот всегда находится в одном из этих состояний
class State(Enum):
    EXIT_POCKET = 1       # Фаза 1: Выезд из кармана назад (на юг)
    TURN_EAST = 2         # Фаза 2: Поворот на восток (90° вправо)
    GO_EAST_ROOM1 = 3     # Фаза 3: Движение на восток в первой комнате
    ENTER_CORRIDOR = 4    # Фаза 4: Вход в коридор между комнатами
    TRAVERSE_CORRIDOR = 5 # Фаза 5: Прохождение коридора
    GO_EAST_ROOM2 = 6     # Фаза 6: Движение на восток во второй комнате
    FIND_FINISH = 7       # Фаза 7: Поиск финиша и заезд на белый прямоугольник
    FINISH = 8            # Фаза 8: Финиш - миссия выполнена!
    AVOID_OBSTACLE = 9    # Временное состояние: объезд препятствия


# ==================== Вспомогательные функции ====================

def angle_wrap(angle):
    """
    Нормализация угла в диапазон [-π, π].

    Углы в радианах циклические: 2π = 0, 3π = π и т.д.
    Эта функция приводит любой угол в диапазон от -π до +π.

    Пример:
        angle_wrap(3.5 * math.pi) → -0.5 * math.pi

    Args:
        angle (float): Угол в радианах

    Returns:
        float: Нормализованный угол в диапазоне [-π, π]
    """
    # Формула: сначала сдвигаем на +π, берём остаток от деления на 2π, потом сдвигаем на -π
    return (angle + math.pi) % (2 * math.pi) - math.pi


def angle_diff(target, current):
    """
    Вычислить кратчайшую разницу между двумя углами.

    Например, если нужно повернуть от 350° к 10°, разница = 20°, а не 340°.

    Args:
        target (float): Целевой угол (куда нужно повернуть)
        current (float): Текущий угол (куда смотрит робот)

    Returns:
        float: Разница углов (положительное = поворот влево, отрицательное = вправо)
    """
    return angle_wrap(target - current)


def dist2d(p1, p2):
    """
    Вычислить евклидово расстояние между двумя точками в 2D.

    Формула: sqrt((x2-x1)^2 + (y2-y1)^2)

    Args:
        p1 (tuple): Первая точка (x1, y1)
        p2 (tuple): Вторая точка (x2, y2)

    Returns:
        float: Расстояние в метрах
    """
    # math.hypot - это эффективная реализация формулы расстояния
    return math.hypot(p1[0] - p2[0], p1[1] - p2[1])


def idx_to_angle(i, n):
    """
    Преобразовать индекс луча лидара в угол.

    Лидар возвращает массив расстояний. Каждый элемент массива соответствует
    определённому углу относительно передней части робота.

    Например, если лидар имеет обзор 90° (±45°) и 100 лучей:
    - Индекс 0 → угол -45° (крайний левый луч)
    - Индекс 50 → угол 0° (центральный луч, прямо вперёд)
    - Индекс 99 → угол +45° (крайний правый луч)

    Args:
        i (int): Индекс луча в массиве
        n (int): Общее количество лучей

    Returns:
        float: Угол луча в радианах относительно передней части робота
    """
    if n <= 1:
        return 0.0  # Если всего один луч, он направлен прямо вперёд

    # Центральный луч имеет индекс n//2, остальные пропорционально распределены
    return (i - n // 2) * (2 * LIDAR_HALF_FOV) / (n - 1)


def smooth_ranges(ranges, window=3):
    """
    Сглаживание данных лидара методом скользящего среднего.

    Лидар иногда выдаёт "шумные" данные (случайные скачки дистанции).
    Эта функция усредняет соседние значения, чтобы данные стали более плавными.

    Пример:
        ranges = [1.0, 5.0, 1.0]  # Шумное значение 5.0 посередине
        smooth_ranges(ranges, 3) → [2.33, 2.33, 3.0]  # Сглажено

    Args:
        ranges (list): Список дистанций от лидара
        window (int): Размер окна сглаживания (3 = усредняем по 3 соседним точкам)

    Returns:
        list: Сглаженный список дистанций
    """
    if window <= 1:
        return ranges  # Если окно = 1, сглаживание не нужно

    smoothed = []           # Результат
    half = window // 2      # Половина окна (для центрирования)
    n = len(ranges)         # Количество измерений

    # Проходим по каждому элементу
    for i in range(n):
        # Определяем границы окна (не выходя за пределы массива)
        start = max(0, i - half)       # Левая граница
        end = min(n, i + half + 1)     # Правая граница

        # Усредняем значения в окне
        avg = sum(ranges[start:end]) / (end - start)
        smoothed.append(avg)

    return smoothed


def get_sector_range(ranges, angle_center, angle_width=math.pi/6):
    """
    Получить минимальную дистанцию в заданном секторе лидара.

    Лидар сканирует пространство вокруг робота. Эта функция выбирает
    определённый сектор (например, "спереди" или "слева") и находит
    ближайшее препятствие в этом секторе.

    Args:
        ranges (list): Массив дистанций от лидара
        angle_center (float): Центральный угол сектора (радианы)
        angle_width (float): Ширина сектора (радианы)

    Returns:
        float: Минимальная дистанция в секторе (метры)

    Пример:
        get_sector_range(ranges, 0.0, math.pi/4) → минимальная дистанция спереди (±22.5°)
    """
    n = len(ranges)
    if n == 0:
        return MAX_RANGE  # Если нет данных, возвращаем максимум

    min_dist = MAX_RANGE  # Начальное значение

    # Проходим по всем лучам лидара
    for i, r in enumerate(ranges):
        # Вычисляем угол этого луча
        angle = idx_to_angle(i, n)

        # Проверяем, попадает ли луч в наш сектор
        if abs(angle - angle_center) <= angle_width / 2:
            # Если да, обновляем минимальную дистанцию
            min_dist = min(min_dist, r)

    return min_dist


def get_front_range(ranges):
    """
    Получить минимальную дистанцию СПЕРЕДИ робота.

    Использует сектор шириной ±22.5° (всего 45°) по центру.

    Args:
        ranges (list): Массив дистанций от лидара

    Returns:
        float: Минимальная дистанция спереди (метры)
    """
    return get_sector_range(ranges, 0.0, math.pi / 4)


def get_left_range(ranges):
    """
    Получить минимальную дистанцию СЛЕВА от робота.

    Использует сектор с центром на +45° (слева) шириной ±15°.

    Args:
        ranges (list): Массив дистанций от лидара

    Returns:
        float: Минимальная дистанция слева (метры)
    """
    return get_sector_range(ranges, math.pi / 4, math.pi / 6)


def get_right_range(ranges):
    """
    Получить минимальную дистанцию СПРАВА от робота.

    Использует сектор с центром на -45° (справа) шириной ±15°.

    Args:
        ranges (list): Массив дистанций от лидара

    Returns:
        float: Минимальная дистанция справа (метры)
    """
    return get_sector_range(ranges, -math.pi / 4, math.pi / 6)


def find_wide_gap(ranges, min_gap_width=0.35, min_depth=0.5):
    """
    Найти самый широкий проход в данных лидара.

    ВАЖНО для маленького робота (20-30 см): ищем непрерывные промежутки,
    где можно проехать, даже если рядом есть близкие препятствия.

    Args:
        ranges (list): Массив дистанций от лидара
        min_gap_width (float): Минимальная ширина прохода в радианах (0.35 рад ≈ 20°)
        min_depth (float): Минимальная глубина прохода в метрах (0.5м)

    Returns:
        tuple: (angle, depth, width) - угол направления прохода, глубина, ширина
               или (None, 0, 0) если проход не найден
    """
    n = len(ranges)
    if n == 0:
        return (None, 0, 0)

    best_gap = (None, 0, 0)  # (angle, depth, width)

    # Сканируем весь диапазон лидара
    i = 0
    while i < n:
        # Если текущая точка достаточно далеко (потенциальный проход)
        if ranges[i] >= min_depth:
            # Находим конец прохода
            gap_start = i
            gap_sum_dist = 0
            gap_count = 0

            while i < n and ranges[i] >= min_depth:
                gap_sum_dist += ranges[i]
                gap_count += 1
                i += 1

            # Вычисляем параметры прохода
            gap_center_idx = (gap_start + i - 1) / 2
            gap_angle = idx_to_angle(int(gap_center_idx), n)
            gap_width = (i - gap_start) / n * (2 * LIDAR_FOV)  # Ширина в радианах
            gap_depth = gap_sum_dist / gap_count  # Средняя глубина

            # Проверяем, лучше ли этот проход
            if gap_width >= min_gap_width:
                # Критерий: приоритет на глубину * ширину (площадь прохода)
                gap_score = gap_depth * gap_width
                best_score = best_gap[1] * best_gap[2] if best_gap[0] is not None else 0

                if gap_score > best_score:
                    best_gap = (gap_angle, gap_depth, gap_width)
        else:
            i += 1

    return best_gap


def get_east_direction_gap(ranges, current_angle):
    """
    Найти проход в направлении ВОСТОКА относительно текущего угла робота.

    Ищет проходы в секторе ±60° от направления на восток.

    Args:
        ranges (list): Массив дистанций от лидара
        current_angle (float): Текущий угол робота (радианы)

    Returns:
        tuple: (relative_angle, depth, width) - угол относительно робота, глубина, ширина
               или (None, 0, 0) если прохода нет
    """
    # Вычисляем, куда нужно повернуть, чтобы смотреть на восток
    target_angle = EAST_ANGLE
    angle_to_east = angle_diff(target_angle, current_angle)

    # Ищем все проходы
    gap_angle, gap_depth, gap_width = find_wide_gap(ranges, min_gap_width=0.3, min_depth=0.5)

    if gap_angle is None:
        return (None, 0, 0)

    # Проверяем, находится ли проход в направлении востока (±60°)
    angle_diff_to_east = abs(angle_diff(gap_angle, 0))  # gap_angle уже в системе координат робота

    # Сравниваем направление прохода с направлением на восток
    if abs(angle_diff(gap_angle, angle_to_east)) < math.radians(60):
        return (gap_angle, gap_depth, gap_width)

    return (None, 0, 0)


# ==================== Сетевое взаимодействие ====================

# Создаём UDP сокет для отправки команд роботу
sock_cmd = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)


def tcp_accept():
    """
    Принять TCP соединение для получения телеметрии.

    TCP - это надёжный протокол с установкой соединения.
    Эта функция создаёт сервер, который ждёт подключения от робота.

    Returns:
        socket: Соединение с роботом
    """
    # Создаём TCP сокет
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

    # Привязываем к адресу и порту
    s.bind((TEL_HOST, TEL_PORT))

    # Начинаем слушать входящие соединения (максимум 1 клиент)
    s.listen(1)

    print(f"[NET] Ожидание подключения на {TEL_HOST}:{TEL_PORT}...")

    # Ждём подключения (эта строка блокирует выполнение, пока робот не подключится)
    conn, _ = s.accept()

    print("[NET] Робот подключён!")
    return conn


# Инициализация сокета для телеметрии (зависит от выбранного протокола)
if PROTO == "udp":
    # UDP - простой протокол без установки соединения
    sock_tel = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock_tel.bind((TEL_HOST, TEL_PORT))
else:
    # TCP - более надёжный, но требует установки соединения
    sock_tel = tcp_accept()


def send_cmd(v, w):
    """
    Отправить команду управления роботу.

    Команда состоит из двух чисел:
    - v (linear velocity) - линейная скорость (м/с)
        Положительная = вперёд, отрицательная = назад
    - w (angular velocity) - угловая скорость (рад/с)
        Положительная = поворот влево, отрицательная = вправо

    Формат пакета: 2 float32 числа (8 байт), little-endian

    Args:
        v (float): Линейная скорость (м/с)
        w (float): Угловая скорость (рад/с)
    """
    # struct.pack упаковывает числа в бинарный формат
    # "<2f" означает: 2 float в little-endian формате
    packet = struct.pack("<2f", float(v), float(w))

    # Отправляем пакет по UDP на робота
    sock_cmd.sendto(packet, (CMD_HOST, CMD_PORT))


def recv_all(sock, n):
    """
    Получить ровно n байт по TCP.

    TCP может отправлять данные частями, поэтому нужно читать в цикле,
    пока не получим все n байт.

    Args:
        sock (socket): TCP сокет
        n (int): Количество байт для чтения

    Returns:
        bytes: Прочитанные данные (или None при ошибке)
    """
    buf = b""  # Буфер для накопления данных (пустая строка байтов)

    while len(buf) < n:
        # Пытаемся прочитать оставшиеся байты
        chunk = sock.recv(n - len(buf))

        # Если recv вернул пустую строку, соединение закрыто
        if not chunk:
            return None

        # Добавляем полученные данные в буфер
        buf += chunk

    return buf


def recv_telemetry():
    """
    Получить пакет телеметрии от робота.

    Формат пакета (протокол WBTG):
    1. Заголовок: b"WBTG" (4 байта)
    2. Одометрия: 9 float32 чисел (36 байт):
       - x, y, th - позиция и угол робота
       - vx, vy, vth - скорости (линейные и угловая)
       - wx, wy, wz - угловые скорости по осям (гироскоп)
    3. Количество точек лидара: N (uint32, 4 байта)
    4. Массив дистанций лидара: N float32 чисел

    Returns:
        dict: Словарь с данными телеметрии:
            {
                'x': float, 'y': float, 'th': float,  # Позиция и угол
                'vx': float, 'vy': float, 'vth': float,  # Скорости
                'wx': float, 'wy': float, 'wz': float,  # Гироскоп
                'ranges': list  # Массив дистанций лидара
            }
        или None при ошибке
    """
    # Получаем данные (зависит от протокола)
    if PROTO == "udp":
        # UDP: получаем весь пакет сразу
        data, _ = sock_tel.recvfrom(65535)  # 65535 - максимальный размер UDP пакета
    else:
        # TCP: сначала читаем размер пакета (4 байта)
        size_bytes = sock_tel.recv(4)
        if not size_bytes:
            return None  # Соединение закрыто

        # Распаковываем размер
        size = struct.unpack("<I", size_bytes)[0]  # "<I" = uint32, little-endian

        # Читаем сам пакет
        data = recv_all(sock_tel, size)

    # Проверяем заголовок
    if not data or not data.startswith(b"WBTG"):
        return None  # Неверный формат

    # Парсим одометрию (9 float32 после заголовка)
    header_size = 4 + 9 * 4  # 4 байта заголовок + 36 байт одометрия
    odom = struct.unpack("<9f", data[4:header_size])  # "<9f" = 9 float, little-endian

    # Распаковываем одометрию
    x, y, th = odom[0], odom[1], odom[2]      # Позиция (x, y) и угол (th)
    vx, vy, vth = odom[3], odom[4], odom[5]   # Скорости
    wx, wy, wz = odom[6], odom[7], odom[8]    # Гироскоп (угловые скорости по осям)

    # Парсим количество точек лидара
    n_points = struct.unpack("<I", data[header_size:header_size + 4])[0]

    # Парсим массив дистанций лидара
    ranges = []
    if n_points > 0:
        # Читаем n_points float32 чисел
        ranges_raw = struct.unpack(
            f"<{n_points}f",
            data[header_size + 4:header_size + 4 + 4 * n_points]
        )

        # Обрабатываем данные:
        # - Если дистанция <= 0, заменяем на MAX_RANGE (нет замера)
        # - Ограничиваем максимальную дистанцию MAX_RANGE
        ranges = [
            MAX_RANGE if r <= 0.0 else min(MAX_RANGE, r)
            for r in ranges_raw
        ]

    # Возвращаем всё в виде словаря
    return {
        'x': x, 'y': y, 'th': th,
        'vx': vx, 'vy': vy, 'vth': vth,
        'wx': wx, 'wy': wy, 'wz': wz,
        'ranges': ranges
    }


# ==================== Класс робота ====================

class Robot:
    """
    Класс, управляющий роботом.

    Робот - это конечный автомат (state machine), который переключается
    между различными состояниями (фазами) в зависимости от текущей ситуации.

    Attributes:
        state (State): Текущее состояние робота
        start_time (float): Время запуска программы
        last_move_time (float): Время последнего движения (для детекции застревания)
        start_pos (tuple): Начальная позиция робота (x, y)
        target_angle (float): Целевой угол для поворота
        target_pos (tuple): Целевая позиция для движения
        in_room2 (bool): Флаг: робот во второй комнате
        avoid_direction (int): Направление объезда препятствия (1 = влево, -1 = вправо)
    """

    def __init__(self):
        """
        Инициализация робота.

        Устанавливает начальное состояние и обнуляет все переменные.
        """
        # Начальное состояние: выезд из кармана
        self.state = State.EXIT_POCKET

        # Временные метки
        self.start_time = time.time()      # Время запуска
        self.last_move_time = time.time()  # Время последнего движения

        # Начальная позиция и цели
        self.start_pos = None      # Будет установлена при первом получении телеметрии
        self.target_angle = None   # Целевой угол для поворота
        self.target_pos = None     # Целевая позиция для движения

        # История для детекции застревания
        self.last_pos = None  # Предыдущая позиция
        self.last_th = None   # Предыдущий угол
        self.last_x = None    # Предыдущая X-координата

        # Флаги прогресса
        self.in_corridor = False       # Флаг: робот в коридоре
        self.passed_corridor = False   # Флаг: робот прошёл коридор
        self.in_room2 = False          # Флаг: робот во второй комнате

        # Для объезда препятствий
        self.avoid_start_time = None   # Время начала объезда
        self.avoid_direction = 1       # Направление объезда (1 = влево, -1 = вправо)

    def update(self, tel):
        """
        Главная функция обновления состояния робота.

        Вызывается на каждой итерации главного цикла. Анализирует телеметрию,
        проверяет условия переключения состояний и вызывает соответствующие
        функции-обработчики.

        Args:
            tel (dict): Словарь с телеметрией от робота

        Returns:
            State: Текущее состояние после обновления
        """
        # Извлекаем данные из телеметрии
        x, y, th = tel['x'], tel['y'], tel['th']  # Позиция и угол
        # КРИТИЧЕСКИ ВАЖНО: нормализуем угол из телеметрии!
        th = angle_wrap(th)
        tel['th'] = th  # Обновляем в словаре
        ranges = smooth_ranges(tel['ranges'], 5)  # Сглаживаем данные лидара

        # === Инициализация начальной позиции ===
        if self.start_pos is None:
            global NORTH_ANGLE, EAST_ANGLE
            self.start_pos = (x, y)
            NORTH_ANGLE = th  # Запоминаем начальный угол как "север"

            # Определяем "восток": в системе координат робота
            # NORTH = 0° → движение по -Y
            # EAST должен быть движение по +X
            # Из анализа логов: th=90° → движение по +Y (неправильно!)
            # Нужно: EAST_ANGLE = NORTH_ANGLE - π/2 = -90° (поворот ВПРАВО)
            EAST_ANGLE = angle_wrap(NORTH_ANGLE - math.pi / 2)  # Поворот ВПРАВО на 90°

            print(f"[INIT] Начальная позиция: ({x:.2f}, {y:.2f})")
            print(f"[INIT] СЕВЕР (начальный угол): {math.degrees(NORTH_ANGLE):.1f}°")
            print(f"[INIT] ВОСТОК (целевое направление): {math.degrees(EAST_ANGLE):.1f}°")
            print(f"[INIT] Калибровка: поворот ВПРАВО на 90°")

        # === Детекция движения ===
        # Обновляем время последнего движения если робот сдвинулся (>2см) ИЛИ повернулся (>2°)
        if self.last_pos is not None and self.last_th is not None:
            pos_changed = dist2d((x, y), self.last_pos) > 0.02
            angle_changed = abs(angle_diff(th, self.last_th)) > math.radians(2)
            if pos_changed or angle_changed:
                self.last_move_time = time.time()
        self.last_pos = (x, y)
        self.last_th = th

        # === Проверка таймаутов ===
        elapsed = time.time() - self.start_time
        if elapsed > TIMEOUT_MAX:
            print(f"[TIMEOUT] Превышено максимальное время {TIMEOUT_MAX}с!")
            print(f"[TIMEOUT] Миссия не выполнена за отведенное время")
            send_cmd(0.0, 0.0)  # Останавливаем робота
            # Переходим в состояние FINISH, чтобы главный цикл завершился
            self.state = State.FINISH
            return self.state

        # === Диспетчер состояний ===
        # В зависимости от текущего состояния вызываем соответствующую функцию
        if self.state == State.EXIT_POCKET:
            self.exit_pocket(tel, ranges)
        elif self.state == State.TURN_EAST:
            self.turn_east(tel)
        elif self.state == State.GO_EAST_ROOM1:
            self.go_east_room1(tel, ranges)
        elif self.state == State.ENTER_CORRIDOR:
            self.enter_corridor(tel, ranges)
        elif self.state == State.TRAVERSE_CORRIDOR:
            self.traverse_corridor(tel, ranges)
        elif self.state == State.GO_EAST_ROOM2:
            self.go_east_room2(tel, ranges)
        elif self.state == State.FIND_FINISH:
            self.find_finish(tel, ranges)
        elif self.state == State.FINISH:
            send_cmd(0.0, 0.0)  # Стоим на месте
            print("[FINISH] Миссия выполнена!")
        elif self.state == State.AVOID_OBSTACLE:
            self.avoid_obstacle(tel, ranges)

        return self.state

    def exit_pocket(self, tel, ranges):
        """
        Фаза 1: Выезд из кармана ВПЕРЕД.

        НОВАЯ СТРАТЕГИЯ (предложена пользователем):
        1. Робот начинает в "кармане" из утят, морда на СЕВЕР (вглубь кармана)
        2. Сначала РАЗВОРАЧИВАЕТСЯ на 180° (морда на ЮГ)
        3. Едет ВПЕРЕД до южной стены
        4. Поворачивает на 90° вправо (на ВОСТОК)
        5. Перед ним - широкая дырка между уткой и южной стеной!

        Преимущество: робот сразу видит свободный проход на восток,
        не нужно объезжать паллеты в тесноте.

        Args:
            tel (dict): Телеметрия
            ranges (list): Данные лидара
        """
        x, y, th = tel['x'], tel['y'], tel['th']

        # Инициализируем подфазу (если ещё не было)
        if not hasattr(self, 'exit_phase'):
            self.exit_phase = 'turn_south'  # Фазы: turn_south → drive_south
            print(f"[EXIT_POCKET] ===== НАЧАЛО ВЫЕЗДА =====")
            print(f"[EXIT_POCKET] Стратегия: разворот 180° → выезд вперед → поворот на восток")

        # Дистанции для лога
        front = get_front_range(ranges)
        left = get_left_range(ranges)
        right = get_right_range(ranges)

        # === ПОДФАЗА 1: РАЗВОРОТ НА 180° (морда на ЮГ) ===
        if self.exit_phase == 'turn_south':
            # Целевой угол: ЮГ = СЕВЕР + 180° = NORTH_ANGLE + π
            south_angle = angle_wrap(NORTH_ANGLE + math.pi)
            angle_error = angle_diff(south_angle, th)

            # Если развернулись (ошибка < 10°), переходим к движению вперед
            if abs(angle_error) < math.radians(10):
                self.exit_phase = 'drive_south'
                self.exit_start_pos = (x, y)  # Запоминаем позицию начала движения
                print(f"[EXIT_POCKET] Развернулся на ЮГ! th={math.degrees(th):.0f}° → начинаю движение вперед")
            else:
                # Поворачиваемся на месте
                w = max(-W_MAX, min(W_MAX, 2.0 * angle_error))
                send_cmd(0.0, w)
                print(f"[EXIT_POCKET] РАЗВОРОТ pos=({x:.2f},{y:.2f}) th={math.degrees(th):.0f}° target={math.degrees(south_angle):.0f}° error={math.degrees(angle_error):.1f}° w={w:.2f}")
            return

        # === ПОДФАЗА 2: ДВИЖЕНИЕ ВПЕРЕД (на юг до стены) ===
        if self.exit_phase == 'drive_south':
            # Вычисляем, сколько проехали от начала движения
            dist_moved = dist2d((x, y), self.exit_start_pos)

            # Условие завершения: уперлись в стену впереди ИЛИ проехали минимум
            if front < OBSTACLE_DIST or dist_moved >= EXIT_POCKET_DIST:
                send_cmd(0.0, 0.0)  # Останавливаемся
                delattr(self, 'exit_phase')  # Очищаем подфазу
                delattr(self, 'exit_start_pos')
                self.state = State.TURN_EAST  # Переключаемся на поворот на восток
                print(f"[EXIT_POCKET] ЗАВЕРШЕНО! Проехали {dist_moved:.2f}м, front={front:.2f}м")
                print(f"[EXIT_POCKET] Позиция: ({x:.2f}, {y:.2f}), угол: {math.degrees(th):.1f}°")
                print(f"[EXIT_POCKET] → TURN_EAST (поворот на восток)")
            else:
                # Едем вперед с нормальной скоростью
                send_cmd(V_NORMAL, 0.0)
                print(f"[EXIT_POCKET] ВПЕРЕД pos=({x:.2f},{y:.2f}) th={math.degrees(th):.0f}° moved={dist_moved:.2f}m F={front:.2f} L={left:.2f} R={right:.2f}")

    def turn_east(self, tel):
        """
        Фаза 2: Поворот на восток.

        После выезда из кармана робот повернут на ЮГ (развернулся 180° и выехал вперед).
        Нужно повернуть на ВОСТОК на 90° ВПРАВО (по часовой стрелке).
        После этого перед роботом будет широкий проход между уткой и южной стеной!

        Args:
            tel (dict): Телеметрия
        """
        x, y, th = tel['x'], tel['y'], tel['th']

        # Устанавливаем целевой угол (только один раз)
        if self.target_angle is None:
            # Используем калиброванный угол "восток"
            self.target_angle = EAST_ANGLE
            print(f"[TURN_EAST] Начало поворота")
            print(f"[TURN_EAST] Текущий угол: {math.degrees(th):.1f}°")
            print(f"[TURN_EAST] Целевой угол (ВОСТОК): {math.degrees(self.target_angle):.1f}°")

        # Вычисляем ошибку угла (насколько нужно ещё повернуть)
        angle_error = angle_diff(self.target_angle, th)

        # Условие завершения: ошибка меньше 0.15 радиан (~8.5°) - увеличили порог для стабильности
        if abs(angle_error) < 0.15:
            send_cmd(0.0, 0.0)  # Останавливаемся
            self.state = State.GO_EAST_ROOM1  # Переключаемся на движение
            self.last_x = tel['x']  # Запоминаем текущую X-координату
            self.last_move_time = time.time()  # СБРАСЫВАЕМ таймер застревания
            print(f"[TURN_EAST] ЗАВЕРШЕНО! Итоговый угол: {math.degrees(th):.1f}°")
            print(f"[TURN_EAST] Позиция: ({x:.2f}, {y:.2f})")
            print(f"[TURN_EAST] → GO_EAST_ROOM1 (движение на восток)")
        else:
            # Поворачиваем: w пропорционально ошибке
            w = max(-W_MAX, min(W_MAX, 2.0 * angle_error))
            send_cmd(0.0, w)  # Поворот на месте (v=0)
            print(f"[TURN_EAST] pos=({x:.2f},{y:.2f}) th={math.degrees(th):.0f}° target={math.degrees(self.target_angle):.0f}° error={math.degrees(angle_error):.1f}° w={w:.2f}")

    def go_east_room1(self, tel, ranges):
        """
        Фаза 3: Движение на восток в первой комнате.

        Робот движется на восток (увеличивая X-координату), объезжая
        препятствия (утят). Цель: найти вход в коридор на восточной стене.

        Args:
            tel (dict): Телеметрия
            ranges (list): Данные лидара
        """
        x, y, th = tel['x'], tel['y'], tel['th']

        # Получаем дистанции в разных направлениях
        front = get_front_range(ranges)  # Спереди
        left = get_left_range(ranges)    # Слева
        right = get_right_range(ranges)  # Справа

        # === Проверка на коридор ===
        # Если лидар детектирует узкий проход (коридор), переключаемся
        if self.detect_corridor(ranges):
            self.state = State.ENTER_CORRIDOR
            print(f"[GO_EAST_ROOM1] Обнаружен коридор → ENTER_CORRIDOR")
            return

        # === Проверка на препятствие СПРАВА (слишком близко к стене) ===
        # Если справа стена ближе 0.7м, отворачиваем ВЛЕВО
        if right < 0.7:
            # Если слишком близко (<0.35м), это критично - переходим в режим объезда
            if right < 0.35:
                print(f"[GO_EAST_ROOM1] КРИТИЧЕСКИ близко к правой стене R={right:.2f}m → AVOID_OBSTACLE")
                self.state = State.AVOID_OBSTACLE
                self.avoid_start_time = time.time()
                self.avoid_direction = 1  # Влево
                return

            # Едем вперёд, добавляя коррекцию влево (не меняя целевой угол резко)
            target_angle = EAST_ANGLE
            angle_error = angle_diff(target_angle, th)
            # Добавляем коррекцию влево пропорционально близости стены
            wall_correction = (0.7 - right) * 2.5  # Чем ближе, тем сильнее влево
            w = max(-W_MAX, min(W_MAX, 1.5 * angle_error + wall_correction))
            v = V_SLOW
            send_cmd(v, w)
            print(f"[GO_EAST_ROOM1] WALL_R! pos=({x:.2f},{y:.2f}) th={math.degrees(th):.0f}° F={front:.2f} L={left:.2f} R={right:.2f} v={v:.2f} w={w:.2f}")
            return

        # === Проверка на препятствие СЛЕВА (слишком близко к стене) ===
        # Если слева стена ближе 0.7м, отворачиваем ВПРАВО
        if left < 0.7:
            # Если слишком близко (<0.35м), это критично - переходим в режим объезда
            if left < 0.35:
                print(f"[GO_EAST_ROOM1] КРИТИЧЕСКИ близко к левой стене L={left:.2f}m → AVOID_OBSTACLE")
                self.state = State.AVOID_OBSTACLE
                self.avoid_start_time = time.time()
                self.avoid_direction = -1  # Вправо
                return

            # Едем вперёд, добавляя коррекцию вправо
            target_angle = EAST_ANGLE
            angle_error = angle_diff(target_angle, th)
            # Добавляем коррекцию вправо пропорционально близости стены
            wall_correction = -(0.7 - left) * 2.5  # Чем ближе, тем сильнее вправо
            w = max(-W_MAX, min(W_MAX, 1.5 * angle_error + wall_correction))
            v = V_SLOW
            send_cmd(v, w)
            print(f"[GO_EAST_ROOM1] WALL_L! pos=({x:.2f},{y:.2f}) th={math.degrees(th):.0f}° F={front:.2f} L={left:.2f} R={right:.2f} v={v:.2f} w={w:.2f}")
            return

        # === Проверка на препятствие ВПЕРЕДИ ===
        # Если впереди близко препятствие, нужно объехать
        if front < OBSTACLE_DIST:
            # НО СНАЧАЛА! Проверяем, есть ли узкий проход на восток рядом
            # (робот маленький 20-30 см, может проехать между паллетами)
            gap_angle, gap_depth, gap_width = get_east_direction_gap(ranges, th)

            if gap_angle is not None and gap_depth > 0.5:
                # Нашли проход на восток! Поворачиваем к нему вместо объезда
                angle_error_to_gap = angle_diff(gap_angle, 0)
                w = max(-W_MAX, min(W_MAX, 2.5 * angle_error_to_gap))

                # Если угол небольшой - едем вперёд, иначе поворачиваем
                if abs(angle_error_to_gap) < math.radians(20):
                    v = V_SLOW
                    send_cmd(v, w)
                    print(f"[GO_EAST_ROOM1] Вижу узкий проход на ВОСТОК! angle={math.degrees(gap_angle):.0f}° depth={gap_depth:.2f}m width={math.degrees(gap_width):.0f}° → еду через проход")
                else:
                    send_cmd(V_SLOW * 0.3, w)
                    print(f"[GO_EAST_ROOM1] Поворот к узкому проходу angle={math.degrees(gap_angle):.0f}° error={math.degrees(angle_error_to_gap):.1f}°")
                return

            # Прохода нет - переходим в режим объезда
            self.state = State.AVOID_OBSTACLE
            self.avoid_start_time = time.time()
            # Выбираем направление объезда: в сторону, где больше свободного места
            self.avoid_direction = 1 if left > right else -1
            print(f"[GO_EAST_ROOM1] Препятствие впереди F={front:.2f}m, узкого прохода нет → AVOID_OBSTACLE")
            return

        # === Движение на восток ===
        # Целевой угол: ВОСТОК (определён при калибровке)
        target_angle = EAST_ANGLE
        angle_error = angle_diff(target_angle, th)

        # Угловая скорость пропорциональна ошибке угла
        w = max(-W_MAX, min(W_MAX, 2.5 * angle_error))

        # Если ошибка угла большая (> 30°), ПОВОРАЧИВАЕМ С МЕДЛЕННЫМ ДВИЖЕНИЕМ ВПЕРЁД
        # (вместо поворота на месте), чтобы не триггерить детекцию застревания
        if abs(angle_error) > math.radians(30):
            # Двигаемся вперёд медленно, одновременно поворачивая (спиральное движение)
            # Это гарантирует, что позиция меняется и робот не считается застрявшим
            v = V_SLOW * 0.5  # Очень медленно вперёд (0.125 м/с)
            send_cmd(v, w)
            print(f"[GO_EAST_ROOM1] ПОВОРОТ! pos=({x:.2f},{y:.2f}) th={math.degrees(th):.0f}° target={math.degrees(EAST_ANGLE):.0f}° error={math.degrees(angle_error):.1f}° v={v:.2f} w={w:.2f}")
            return

        # Линейная скорость зависит от свободного пространства впереди
        if front > SAFE_DIST:
            v = V_FAST  # Быстро, когда путь свободен
        else:
            v = V_SLOW  # Медленно, когда близко препятствие

        # Замедляемся при резких поворотах (но не слишком сильно)
        if abs(w) > W_MAX * 0.5:  # Если поворот резкий
            v *= 0.6  # Замедляемся до 60%

        send_cmd(v, w)
        print(f"[GO_EAST_ROOM1] pos=({x:.2f},{y:.2f}) th={math.degrees(th):.0f}° target={math.degrees(EAST_ANGLE):.0f}° F={front:.2f} L={left:.2f} R={right:.2f} v={v:.2f} w={w:.2f}")

    def detect_corridor(self, ranges):
        """
        Детекция коридора.

        Коридор - это узкий проход с стенами по бокам:
        - Впереди свободно (можно проехать)
        - Слева и справа узко (стены близко)

        Args:
            ranges (list): Данные лидара

        Returns:
            bool: True, если детектирован коридор
        """
        front = get_front_range(ranges)
        left = get_left_range(ranges)
        right = get_right_range(ranges)

        # Условие: впереди далеко, по бокам близко
        corridor_condition = (
            front > 1.5 and
            left < CORRIDOR_WIDTH_MAX and
            right < CORRIDOR_WIDTH_MAX
        )

        return corridor_condition

    def enter_corridor(self, tel, ranges):
        """
        Фаза 4: Вход и прохождение коридора.

        Робот едет по коридору, держась примерно по центру между стенами.
        Цель: выехать во вторую комнату.

        Args:
            tel (dict): Телеметрия
            ranges (list): Данные лидара
        """
        x, y, th = tel['x'], tel['y'], tel['th']

        front = get_front_range(ranges)
        left = get_left_range(ranges)
        right = get_right_range(ranges)

        # === Проверка на выход из коридора ===
        # Если по бокам стало просторно - мы во второй комнате
        if left > ROOM_WIDTH_MIN and right > ROOM_WIDTH_MIN:
            self.state = State.GO_EAST_ROOM2
            self.in_room2 = True
            print(f"[ENTER_CORRIDOR] Вышел во вторую комнату → GO_EAST_ROOM2")
            return

        # === Движение по коридору ===
        # Целевой угол: прямо на восток
        target_angle = EAST_ANGLE
        angle_error = angle_diff(target_angle, th)

        # Коррекция по стенкам: если правая стена ближе, подруливаем влево
        wall_correction = (right - left) * 0.8

        # Общая угловая скорость: коррекция угла + коррекция по стенам
        w = max(-W_MAX, min(W_MAX, 2.0 * angle_error + wall_correction))

        # Линейная скорость
        v = V_NORMAL if front > SAFE_DIST else V_SLOW

        send_cmd(v, w)
        print(f"[ENTER_CORRIDOR] X={x:.2f} F={front:.2f} L={left:.2f} R={right:.2f}")

    def traverse_corridor(self, tel, ranges):
        """
        Фаза 5: Прохождение коридора.

        В данной реализации объединена с enter_corridor.

        Args:
            tel (dict): Телеметрия
            ranges (list): Данные лидара
        """
        self.enter_corridor(tel, ranges)

    def go_east_room2(self, tel, ranges):
        """
        Фаза 6: Движение на восток во второй комнате.

        Робот продолжает движение на восток, объезжая препятствия.
        Цель: найти финишную зону (белый прямоугольник).

        Args:
            tel (dict): Телеметрия
            ranges (list): Данные лидара
        """
        x, y, th = tel['x'], tel['y'], tel['th']

        front = get_front_range(ranges)
        left = get_left_range(ranges)
        right = get_right_range(ranges)

        # === Проверка на финишную зону ===
        # Финиш - это открытое пространство (белый прямоугольник на полу)
        if self.detect_finish_zone(ranges):
            self.state = State.FIND_FINISH
            self.target_pos = (x + 1.0, y)  # Целевая позиция: 1м вперёд
            print(f"[GO_EAST_ROOM2] Обнаружена финишная зона → FIND_FINISH")
            return

        # === Проверка на препятствие СПРАВА ===
        if right < 0.7:
            if right < 0.35:
                print(f"[GO_EAST_ROOM2] КРИТИЧЕСКИ близко к правой стене R={right:.2f}m → AVOID_OBSTACLE")
                self.state = State.AVOID_OBSTACLE
                self.avoid_start_time = time.time()
                self.avoid_direction = 1
                return

            target_angle = EAST_ANGLE
            angle_error = angle_diff(target_angle, th)
            wall_correction = (0.7 - right) * 2.5
            w = max(-W_MAX, min(W_MAX, 1.5 * angle_error + wall_correction))
            v = V_SLOW
            send_cmd(v, w)
            print(f"[GO_EAST_ROOM2] WALL_R! pos=({x:.2f},{y:.2f}) th={math.degrees(th):.0f}° F={front:.2f} L={left:.2f} R={right:.2f} v={v:.2f} w={w:.2f}")
            return

        # === Проверка на препятствие СЛЕВА ===
        if left < 0.7:
            if left < 0.35:
                print(f"[GO_EAST_ROOM2] КРИТИЧЕСКИ близко к левой стене L={left:.2f}m → AVOID_OBSTACLE")
                self.state = State.AVOID_OBSTACLE
                self.avoid_start_time = time.time()
                self.avoid_direction = -1
                return

            target_angle = EAST_ANGLE
            angle_error = angle_diff(target_angle, th)
            wall_correction = -(0.7 - left) * 2.5
            w = max(-W_MAX, min(W_MAX, 1.5 * angle_error + wall_correction))
            v = V_SLOW
            send_cmd(v, w)
            print(f"[GO_EAST_ROOM2] WALL_L! pos=({x:.2f},{y:.2f}) th={math.degrees(th):.0f}° F={front:.2f} L={left:.2f} R={right:.2f} v={v:.2f} w={w:.2f}")
            return

        # === Проверка на препятствие ВПЕРЕДИ ===
        if front < OBSTACLE_DIST:
            # НО СНАЧАЛА! Проверяем, есть ли узкий проход на восток рядом
            gap_angle, gap_depth, gap_width = get_east_direction_gap(ranges, th)

            if gap_angle is not None and gap_depth > 0.5:
                # Нашли проход на восток! Поворачиваем к нему вместо объезда
                angle_error_to_gap = angle_diff(gap_angle, 0)
                w = max(-W_MAX, min(W_MAX, 2.5 * angle_error_to_gap))

                if abs(angle_error_to_gap) < math.radians(20):
                    v = V_SLOW
                    send_cmd(v, w)
                    print(f"[GO_EAST_ROOM2] Вижу узкий проход на ВОСТОК! angle={math.degrees(gap_angle):.0f}° depth={gap_depth:.2f}m → еду через проход")
                else:
                    send_cmd(V_SLOW * 0.3, w)
                    print(f"[GO_EAST_ROOM2] Поворот к узкому проходу angle={math.degrees(gap_angle):.0f}° error={math.degrees(angle_error_to_gap):.1f}°")
                return

            # Прохода нет - переходим в режим объезда
            self.state = State.AVOID_OBSTACLE
            self.avoid_start_time = time.time()
            self.avoid_direction = 1 if left > right else -1
            print(f"[GO_EAST_ROOM2] Препятствие впереди F={front:.2f}m, узкого прохода нет → AVOID_OBSTACLE")
            return

        # === Движение на восток ===
        target_angle = EAST_ANGLE
        angle_error = angle_diff(target_angle, th)
        w = max(-W_MAX, min(W_MAX, 2.5 * angle_error))

        # Если ошибка угла большая (> 30°), ПОВОРАЧИВАЕМ С МЕДЛЕННЫМ ДВИЖЕНИЕМ ВПЕРЁД
        # (вместо поворота на месте), чтобы не триггерить детекцию застревания
        if abs(angle_error) > math.radians(30):
            # Двигаемся вперёд медленно, одновременно поворачивая (спиральное движение)
            v = V_SLOW * 0.5  # Очень медленно вперёд (0.125 м/с)
            send_cmd(v, w)
            print(f"[GO_EAST_ROOM2] ПОВОРОТ! pos=({x:.2f},{y:.2f}) th={math.degrees(th):.0f}° target={math.degrees(EAST_ANGLE):.0f}° error={math.degrees(angle_error):.1f}° v={v:.2f} w={w:.2f}")
            return

        v = V_FAST if front > SAFE_DIST else V_SLOW
        if abs(w) > W_MAX * 0.5:
            v *= 0.6

        send_cmd(v, w)
        print(f"[GO_EAST_ROOM2] pos=({x:.2f},{y:.2f}) th={math.degrees(th):.0f}° F={front:.2f} L={left:.2f} R={right:.2f} v={v:.2f} w={w:.2f}")

    def detect_finish_zone(self, ranges):
        """
        Детекция финишной зоны.

        Финиш - это открытое пространство (много свободного места вокруг).

        Args:
            ranges (list): Данные лидара

        Returns:
            bool: True, если детектирована финишная зона
        """
        front = get_front_range(ranges)
        left = get_left_range(ranges)
        right = get_right_range(ranges)

        # Условие: много свободного пространства со всех сторон
        return (
            front > FINISH_ZONE_OPEN and
            left > FINISH_ZONE_OPEN and
            right > FINISH_ZONE_OPEN
        )

    def find_finish(self, tel, ranges):
        """
        Фаза 7: Поиск и заезд на финиш.

        Робот находится в открытом пространстве финишной зоны.
        Нужно проехать вперёд и остановиться на белом прямоугольнике.

        Args:
            tel (dict): Телеметрия
            ranges (list): Данные лидара
        """
        x, y, th = tel['x'], tel['y'], tel['th']

        front = get_front_range(ranges)

        # Проверяем, что всё ещё в открытом пространстве
        if front > FINISH_ZONE_OPEN:
            # Едем прямо вперёд
            target_angle = EAST_ANGLE
            angle_error = angle_diff(target_angle, th)
            w = max(-W_MAX, min(W_MAX, 2.0 * angle_error))

            send_cmd(V_SLOW, w)
            print(f"[FIND_FINISH] pos=({x:.2f},{y:.2f}) th={math.degrees(th):.0f}° F={front:.2f}")

            # Если достигли целевой позиции - финиш!
            if self.target_pos and dist2d((x, y), self.target_pos) < FINISH_RADIUS:
                self.state = State.FINISH
                send_cmd(0.0, 0.0)
                print(f"[FIND_FINISH] Финиш достигнут!")
        else:
            # Если вдруг снова появились препятствия, возвращаемся к движению
            self.state = State.GO_EAST_ROOM2
            print(f"[FIND_FINISH] Ещё препятствия, возврат → GO_EAST_ROOM2")

    def avoid_obstacle(self, tel, ranges):
        """
        Временное состояние: объезд препятствия.

        СТРАТЕГИЯ:
        1. Отъехать НАЗАД на безопасное расстояние (0.3м минимум)
        2. Повернуть в сторону свободного пространства (90°)
        3. Проехать ВПЕРЁД, огибая препятствие
        4. Вернуться к движению на ВОСТОК

        Args:
            tel (dict): Телеметрия
            ranges (list): Данные лидара
        """
        x, y, th = tel['x'], tel['y'], tel['th']

        front = get_front_range(ranges)
        left = get_left_range(ranges)
        right = get_right_range(ranges)

        # Проверка таймаута объезда (максимум 10 секунд на один объезд)
        if hasattr(self, 'avoid_start_time') and self.avoid_start_time is not None:
            avoid_elapsed = time.time() - self.avoid_start_time
            if avoid_elapsed > 10.0:
                print(f"[AVOID_OBSTACLE] ТАЙМАУТ {avoid_elapsed:.1f}с! Принудительный возврат к движению")
                # Очищаем все состояния
                if hasattr(self, 'avoid_phase'):
                    delattr(self, 'avoid_phase')
                if hasattr(self, 'avoid_backup_start'):
                    delattr(self, 'avoid_backup_start')
                if hasattr(self, 'avoid_turn_target'):
                    delattr(self, 'avoid_turn_target')
                if hasattr(self, 'avoid_forward_start'):
                    delattr(self, 'avoid_forward_start')

                # Возврат к основному движению
                if self.in_room2:
                    self.state = State.GO_EAST_ROOM2
                else:
                    self.state = State.GO_EAST_ROOM1
                return

        # Инициализация фазы объезда (если ещё не было)
        if not hasattr(self, 'avoid_phase') or self.avoid_phase is None:
            self.avoid_phase = 'backup'  # Фазы: backup → turn → forward → resume
            self.avoid_backup_start = (x, y)
            self.avoid_turn_target = None
            print(f"[AVOID_OBSTACLE] ===== НАЧАЛО ОБЪЕЗДА =====")
            print(f"[AVOID_OBSTACLE] Позиция: ({x:.2f}, {y:.2f}) th={math.degrees(th):.0f}°")
            print(f"[AVOID_OBSTACLE] Препятствие F={front:.2f}m")
            print(f"[AVOID_OBSTACLE] Свободно: L={left:.2f}m R={right:.2f}m")
            print(f"[AVOID_OBSTACLE] Направление объезда: {'ВЛЕВО' if self.avoid_direction > 0 else 'ВПРАВО'}")
            print(f"[AVOID_OBSTACLE] Backup start: ({self.avoid_backup_start[0]:.2f}, {self.avoid_backup_start[1]:.2f})")

        # === ФАЗА 1: TURN_AWAY (поворачиваем до тех пор, пока впереди не освободится) ===
        if self.avoid_phase == 'backup':
            # Сразу переходим к повороту (без отъезда назад!)
            self.avoid_phase = 'turn'
            print(f"[AVOID_OBSTACLE] Начинаю поворот в сторону {'ВЛЕВО' if self.avoid_direction > 0 else 'ВПРАВО'}")
            return

        # === ФАЗА 2: TURN (поворачиваем, пока впереди не освободится) ===
        if self.avoid_phase == 'turn':
            # Проверяем, достаточно ли свободно впереди
            # ВАЖНО: используем более низкий порог (0.4м вместо SAFE_DIST=0.6м),
            # чтобы робот мог выйти из объезда даже в узких местах
            if front > 0.4:
                # Впереди свободно! Переходим к движению вперёд
                self.avoid_phase = 'forward'
                self.avoid_forward_start = (x, y)
                print(f"[AVOID_OBSTACLE] Впереди свободно F={front:.2f}m! Угол={math.degrees(th):.0f}° → Еду вперёд")
            else:
                # Поворачиваем на месте в выбранном направлении
                w = W_MAX * 0.8 * self.avoid_direction  # 80% от максимальной скорости поворота
                send_cmd(0.0, w)
                print(f"[AVOID_OBSTACLE] TURN pos=({x:.2f},{y:.2f}) th={math.degrees(th):.0f}° F={front:.2f} L={left:.2f} R={right:.2f} w={w:.2f}")
            return

        # === ФАЗА 3: FORWARD (проехать вперёд, огибая препятствие) ===
        if self.avoid_phase == 'forward':
            forward_dist = dist2d((x, y), self.avoid_forward_start)

            # Едем вперёд минимум 0.5м
            if forward_dist >= 0.5:
                # Переход к возврату на курс восток
                self.avoid_phase = 'resume'
                print(f"[AVOID_OBSTACLE] Объехал! Проехал {forward_dist:.2f}m → Возврат на ВОСТОК")
            else:
                # Проверяем препятствия
                if front < OBSTACLE_DIST:
                    # Снова препятствие впереди - повторяем поворот
                    print(f"[AVOID_OBSTACLE] Снова препятствие F={front:.2f}m! → Повторяю поворот")
                    self.avoid_phase = 'turn'
                    return

                # Едем вперёд прямо (без коррекции к востоку)
                v = V_NORMAL if front > SAFE_DIST else V_SLOW
                send_cmd(v, 0.0)
                print(f"[AVOID_OBSTACLE] FORWARD pos=({x:.2f},{y:.2f}) dist={forward_dist:.2f}m/{0.5:.2f}m F={front:.2f} L={left:.2f} R={right:.2f}")
            return

        # === ФАЗА 4: RESUME (вернуться к движению на восток) ===
        if self.avoid_phase == 'resume':
            # Ищем проход в направлении ВОСТОКА
            gap_angle, gap_depth, gap_width = get_east_direction_gap(ranges, th)

            # Если нашли проход на восток - поворачиваем к нему
            if gap_angle is not None:
                angle_error = angle_diff(gap_angle, 0)  # gap_angle в системе координат робота

                # Если почти направлены на проход (< 15°), переходим к движению
                if abs(angle_error) < math.radians(15):
                    # Очищаем состояние объезда
                    delattr(self, 'avoid_phase')
                    delattr(self, 'avoid_backup_start')
                    delattr(self, 'avoid_turn_target')
                    if hasattr(self, 'avoid_forward_start'):
                        delattr(self, 'avoid_forward_start')

                    # Возврат к основному движению
                    if self.in_room2:
                        self.state = State.GO_EAST_ROOM2
                        print(f"[AVOID_OBSTACLE] Нашел проход на восток! angle={math.degrees(gap_angle):.0f}° depth={gap_depth:.2f}m → GO_EAST_ROOM2")
                    else:
                        self.state = State.GO_EAST_ROOM1
                        print(f"[AVOID_OBSTACLE] Нашел проход на восток! angle={math.degrees(gap_angle):.0f}° depth={gap_depth:.2f}m → GO_EAST_ROOM1")
                else:
                    # Поворачиваем к проходу на восток
                    w = max(-W_MAX, min(W_MAX, 2.0 * angle_error))
                    send_cmd(0.0, w)
                    print(f"[AVOID_OBSTACLE] RESUME: поворот к проходу на ВОСТОК angle={math.degrees(gap_angle):.0f}° error={math.degrees(angle_error):.1f}° w={w:.2f}")
            # Если прохода на восток нет, но впереди свободно - возвращаемся к обычному движению
            elif front > SAFE_DIST:
                # Очищаем состояние объезда
                delattr(self, 'avoid_phase')
                delattr(self, 'avoid_backup_start')
                delattr(self, 'avoid_turn_target')
                if hasattr(self, 'avoid_forward_start'):
                    delattr(self, 'avoid_forward_start')

                # Возврат к основному движению (GO_EAST сам повернет на восток)
                if self.in_room2:
                    self.state = State.GO_EAST_ROOM2
                    print(f"[AVOID_OBSTACLE] Проход на восток не найден, но впереди свободно → GO_EAST_ROOM2")
                else:
                    self.state = State.GO_EAST_ROOM1
                    print(f"[AVOID_OBSTACLE] Проход на восток не найден, но впереди свободно → GO_EAST_ROOM1")
            else:
                # Ещё препятствия, продолжаем объезд
                print(f"[AVOID_OBSTACLE] Ещё препятствия F={front:.2f}m, продолжаю forward")
                self.avoid_phase = 'forward'
                self.avoid_forward_start = (x, y)

    def recover_from_stuck(self, ranges):
        """
        Восстановление при застревании.

        УСИЛЕННАЯ ПРОЦЕДУРА:
        1. Отъехать назад на 0.5м
        2. Повернуть в сторону свободного пространства на 120°
        3. Проехать вперёд 0.5м
        4. Вернуться к основному движению

        Args:
            ranges (list): Данные лидара
        """
        front = get_front_range(ranges)
        left = get_left_range(ranges)
        right = get_right_range(ranges)

        print(f"[STUCK RECOVERY] Застрял! F={front:.2f} L={left:.2f} R={right:.2f}")
        print(f"[STUCK RECOVERY] Отъезжаю назад 0.5м...")

        # Едем назад сильнее и дольше
        for _ in range(25):  # ~0.5 сек
            send_cmd(-V_NORMAL, 0.0)
            time.sleep(0.02)

        print(f"[STUCK RECOVERY] Поворачиваю в сторону {'ЛЕВО' if left > right else 'ПРАВО'}...")

        # Поворачиваем на 120° в сторону свободы
        turn_dir = 1 if left > right else -1
        for _ in range(40):  # ~0.8 сек поворота
            send_cmd(0.0, W_MAX * 0.8 * turn_dir)
            time.sleep(0.02)

        print(f"[STUCK RECOVERY] Еду вперёд...")

        # Едем вперёд
        for _ in range(25):  # ~0.5 сек
            send_cmd(V_NORMAL, 0.0)
            time.sleep(0.02)

        print(f"[STUCK RECOVERY] Восстановление завершено, возврат к движению")

        # Сбрасываем таймер
        self.last_move_time = time.time()

        # ПОЛНОСТЬЮ сбрасываем состояние объезда
        if hasattr(self, 'avoid_phase'):
            delattr(self, 'avoid_phase')
        if hasattr(self, 'avoid_backup_start'):
            delattr(self, 'avoid_backup_start')
        if hasattr(self, 'avoid_turn_target'):
            delattr(self, 'avoid_turn_target')
        if hasattr(self, 'avoid_forward_start'):
            delattr(self, 'avoid_forward_start')

        # Возвращаемся к основному движению (НЕ в режим объезда!)
        if self.in_room2:
            self.state = State.GO_EAST_ROOM2
            print(f"[STUCK RECOVERY] → GO_EAST_ROOM2")
        else:
            self.state = State.GO_EAST_ROOM1
            print(f"[STUCK RECOVERY] → GO_EAST_ROOM1")


# ==================== Главный цикл программы ====================

def main():
    """
    Главная функция программы.

    Инициализирует робота и запускает главный цикл управления.
    """
    # Создаём объект робота
    robot = Robot()

    print("=" * 60)
    print("[MAIN] Робот инициализирован")
    print(f"[MAIN] Миссия: пройти лабиринт за {TIMEOUT_MAX}с")
    print("=" * 60)

    try:
        # Главный цикл: работает, пока не достигнем финиша
        while robot.state != State.FINISH:
            # Получаем телеметрию от робота
            tel = recv_telemetry()

            # Если пакет не получен или повреждён, пропускаем итерацию
            if tel is None:
                continue

            # Обновляем состояние робота на основе телеметрии
            robot.update(tel)

            # Небольшая задержка для стабильности (50 Гц = 0.02 сек)
            time.sleep(0.02)

        # === Финиш достигнут (или таймаут)! ===
        elapsed = time.time() - robot.start_time
        print("\n" + "=" * 60)
        if elapsed < TIMEOUT_MAX:
            print(f"[SUCCESS] Миссия выполнена за {elapsed:.1f} секунд!")
        else:
            print(f"[TIMEOUT] Программа остановлена по таймауту ({elapsed:.1f}с)")
        print("=" * 60 + "\n")

        # Удерживаем остановку (отправляем команду стоп несколько раз)
        for _ in range(10):
            send_cmd(0.0, 0.0)
            time.sleep(0.1)

    except KeyboardInterrupt:
        # Обработка прерывания (Ctrl+C)
        print("\n[MAIN] Прервано пользователем")

    finally:
        # Гарантированная остановка и закрытие соединений
        send_cmd(0.0, 0.0)  # Останавливаем робота
        sock_cmd.close()     # Закрываем сокет команд
        sock_tel.close()     # Закрываем сокет телеметрии
        print("[MAIN] Программа завершена")


# ==================== Точка входа ====================

if __name__ == "__main__":
    # Этот блок выполняется только при прямом запуске скрипта
    # (не при импорте как модуль)
    main()