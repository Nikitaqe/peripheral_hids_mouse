/*
 * Copyright (c) 2018 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */
#include <zephyr/types.h>
#include <stddef.h>
#include <string.h>
#include <errno.h>
#include <zephyr/sys/printk.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>
#include <soc.h>
#include <assert.h>

#include <zephyr/settings/settings.h>

#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/hci.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/bluetooth/gatt.h>

#include <zephyr/bluetooth/services/bas.h>
#include <bluetooth/services/hids.h>
#include <zephyr/bluetooth/services/dis.h>
#include <dk_buttons_and_leds.h>



#include <zephyr/shell/shell.h>



#define DEVICE_NAME     CONFIG_BT_DEVICE_NAME
#define DEVICE_NAME_LEN (sizeof(DEVICE_NAME) - 1)

#define BASE_USB_HID_SPEC_VERSION   0x0101

/* Number of pixels by which the cursor is moved when a button is pushed.*/
#define MOVEMENT_SPEED              5
/* Number of input reports in this application. 
Количество отчетов о вводимых данных в этом приложении.*/
#define INPUT_REPORT_COUNT          3
/* Length of Mouse Input Report containing button data. 
Длина отчета о вводимых данных с помощью мыши, содержащего данные о кнопке.*/
#define INPUT_REP_BUTTONS_LEN       3
/* Length of Mouse Input Report containing movement data. 
Длина отчета о вводе данных мышью, содержащего данные о перемещении.*/
#define INPUT_REP_MOVEMENT_LEN      3
/* Length of Mouse Input Report containing media player data. 
Длина отчета о вводе данных мышью, содержащего данные медиаплеера.*/
#define INPUT_REP_MEDIA_PLAYER_LEN  1
/* Index of Mouse Input Report containing button data. 
Индекс отчета о вводе данных с помощью мыши, содержащего данные о кнопках.*/
#define INPUT_REP_BUTTONS_INDEX     0
/* Index of Mouse Input Report containing movement data. 
Индекс отчета о вводе данных мышью, содержащего данные о перемещении.*/
#define INPUT_REP_MOVEMENT_INDEX    1
/* Index of Mouse Input Report containing media player data. 
Индекс отчета о вводе данных мышью, содержащего данные медиаплеера.*/
#define INPUT_REP_MPLAYER_INDEX     2
/* Id of reference to Mouse Input Report containing button data. 
Идентификатор ссылки на отчет о вводе данных с помощью мыши, содержащий данные о кнопках.*/
#define INPUT_REP_REF_BUTTONS_ID    1
/* Id of reference to Mouse Input Report containing movement data. 
Идентификатор ссылки на отчет о вводе данных с помощью мыши, содержащий данные о перемещении.*/
#define INPUT_REP_REF_MOVEMENT_ID   2
/* Id of reference to Mouse Input Report containing media player data. 
Идентификатор ссылки на отчет о вводе мышью, содержащий данные медиаплеера.*/
#define INPUT_REP_REF_MPLAYER_ID    3

/* HIDs queue size. */
#define HIDS_QUEUE_SIZE 10


/* Key used to move cursor left */
#define KEY_LEFT_MASK   DK_BTN1_MSK
/* Key used to move cursor up */
#define KEY_UP_MASK     DK_BTN2_MSK
/* Key used to move cursor right */
#define KEY_RIGHT_MASK  DK_BTN3_MSK
/* Key used to move cursor down */
#define KEY_DOWN_MASK   DK_BTN4_MSK

/* Key used to accept or reject passkey value */
#define KEY_PAIRING_ACCEPT DK_BTN1_MSK
#define KEY_PAIRING_REJECT DK_BTN2_MSK

/* HIDS instance. */
BT_HIDS_DEF(hids_obj,
	    INPUT_REP_BUTTONS_LEN,
	    INPUT_REP_MOVEMENT_LEN,
	    INPUT_REP_MEDIA_PLAYER_LEN);


static struct k_work hids_work;
struct mouse_pos {
	int16_t x_val;
	int16_t y_val;
};

/* Mouse movement queue. */
K_MSGQ_DEFINE(hids_queue,
	      sizeof(struct mouse_pos),
	      HIDS_QUEUE_SIZE,
	      4);

#if CONFIG_BT_DIRECTED_ADVERTISING
/* Bonded address queue. */
K_MSGQ_DEFINE(bonds_queue,
	      sizeof(bt_addr_le_t),
	      CONFIG_BT_MAX_PAIRED,
	      4);
#endif


static const struct bt_data ad[] = {
	BT_DATA_BYTES(BT_DATA_GAP_APPEARANCE,
		      (CONFIG_BT_DEVICE_APPEARANCE >> 0) & 0xff,
		      (CONFIG_BT_DEVICE_APPEARANCE >> 8) & 0xff),
	BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),
	BT_DATA_BYTES(BT_DATA_UUID16_ALL, BT_UUID_16_ENCODE(BT_UUID_HIDS_VAL),
					  BT_UUID_16_ENCODE(BT_UUID_BAS_VAL)),
};

static const struct bt_data sd[] = {
	BT_DATA(BT_DATA_NAME_COMPLETE, DEVICE_NAME, DEVICE_NAME_LEN),
};


static struct conn_mode {
	struct bt_conn *conn;
	bool in_boot_mode;
} conn_mode[CONFIG_BT_HIDS_MAX_CLIENT_COUNT];

static volatile bool is_adv_running;

static struct k_work adv_work;

static struct k_work pairing_work;
struct pairing_data_mitm {
	struct bt_conn *conn;
	unsigned int passkey;
};


K_MSGQ_DEFINE(mitm_queue,
	      sizeof(struct pairing_data_mitm),
	      CONFIG_BT_HIDS_MAX_CLIENT_COUNT,
	      4);

#if CONFIG_BT_DIRECTED_ADVERTISING
// Функция, которая обрабатывает найденные связанные устройства
static void bond_find(const struct bt_bond_info *info, void *user_data)
{
    // Код ошибки
    int err;

    // Фильтруем уже подключенные устройства
    for (size_t i = 0; i < CONFIG_BT_HIDS_MAX_CLIENT_COUNT; i++) {
        // Если устройство уже подключено, пропускаем его
        if (conn_mode[i].conn) {
            // Получаем адрес устройства
            const bt_addr_le_t *dst = bt_conn_get_dst(conn_mode[i].conn);

            // Сравниваем адрес устройства с адресом найденного связанного устройства
            if (!bt_addr_le_cmp(&info->addr, dst)) {
                // Если адреса совпадают, пропускаем устройство
                return;
            }
        }
    }


    // Добавляем адрес найденного связанного устройства в очередь
    err = k_msgq_put(&bonds_queue, (void *) &info->addr, K_NO_WAIT);
    if (err) {
        // Если очередь полна, выводим сообщение об ошибке
        printk("No space in the queue for the bond.\n");
    }
}
#endif

// Функция, которая продолжает рекламные сообщения
static void advertising_continue(void)
{
    // Параметры рекламных сообщений
    struct bt_le_adv_param adv_param;

#if CONFIG_BT_DIRECTED_ADVERTISING
    // Адрес устройства для направленной рекламы
    bt_addr_le_t addr;

    // Если очередь связанных устройств не пуста, получаем адрес устройства
    if (!k_msgq_get(&bonds_queue, &addr, K_NO_WAIT)) {
        // Если рекламные сообщения уже запущены, останавливаем их
        if (is_adv_running) {
            int err = bt_le_adv_stop();
            if (err) {
                printk("Advertising failed to stop (err %d)\n", err);
                return;
            }
            is_adv_running = false;
        }

        // Настройка параметров направленной рекламы
        adv_param = *BT_LE_ADV_CONN_DIR(&addr);
        adv_param.options |= BT_LE_ADV_OPT_DIR_ADDR_RPA;

        // Запуск направленной рекламы
        int err = bt_le_adv_start(&adv_param, NULL, 0, NULL, 0);
        if (err) {
            printk("Directed advertising failed to start (err %d)\n", err);
            return;
        }

        // Выводим сообщение о начале направленной рекламы
        char addr_buf[BT_ADDR_LE_STR_LEN];
        bt_addr_le_to_str(&addr, addr_buf, BT_ADDR_LE_STR_LEN);
        printk("Direct advertising to %s started\n", addr_buf);
    } else
#endif
    {

        // Если очередь связанных устройств пуста, запускаем обычную рекламу
        int err;

        // Если рекламные сообщения уже запущены, выходим
        if (is_adv_running) {
            return;
        }

        // Настройка параметров обычной рекламы
        adv_param = *BT_LE_ADV_CONN;
        adv_param.options |= BT_LE_ADV_OPT_ONE_TIME;

        // Запуск обычной рекламы
        err = bt_le_adv_start(&adv_param, ad, ARRAY_SIZE(ad), sd, ARRAY_SIZE(sd));
        if (err) {
            printk("Advertising failed to start (err %d)\n", err);
            return;
        }

        // Выводим сообщение о начале обычной рекламы
        printk("Regular advertising started\n");
    }

    // Установка флага о запущенных рекламных сообщениях
    is_adv_running = true;
}

// Функция, которая запускает рекламные сообщения
static void advertising_start(void)
{
#if CONFIG_BT_DIRECTED_ADVERTISING
    // Очистка очереди связанных устройств
    k_msgq_purge(&bonds_queue);

    // Поиск связанных устройств
    bt_foreach_bond(BT_ID_DEFAULT, bond_find, NULL);
#endif

    // Запуск задачи рекламных сообщений
    k_work_submit(&adv_work);
}


// Функция, которая обрабатывает задачу рекламных сообщений
static void advertising_process(struct k_work *work)
{
    // Продолжение рекламных сообщений
    advertising_continue();
}

// Функция, которая обрабатывает задачу пары
static void pairing_process(struct k_work *work)
{
    // Код ошибки
    int err;

    // Данные для пары с MITM
    struct pairing_data_mitm pairing_data;

    // Буфер для адреса устройства
    char addr[BT_ADDR_LE_STR_LEN];

    // Получаем данные для пары из очереди
    err = k_msgq_peek(&mitm_queue, &pairing_data);
    if (err) {
        // Если данные не получены, выходим
        return;
    }

    // Преобразуем адрес устройства в строку
    bt_addr_le_to_str(bt_conn_get_dst(pairing_data.conn), addr, sizeof(addr));


    // Выводим информацию о паре
    printk("Passkey for %s: %06u\n", addr, pairing_data.passkey);
    printk("Press Button 1 to confirm, Button 2 to reject.\n");
}


// Функция, которая добавляет объект соединения
static void insert_conn_object(struct bt_conn *conn)
{
    // Ищем свободное место в массиве conn_mode
    for (size_t i = 0; i < CONFIG_BT_HIDS_MAX_CLIENT_COUNT; i++) {
        // Если найдено свободное место, добавляем объект соединения
        if (!conn_mode[i].conn) {
            conn_mode[i].conn = conn;
            conn_mode[i].in_boot_mode = false;

            // Выходим из функции, поскольку объект соединения добавлен
            return;
        }
    }

    // Если не найдено свободное место, выводим сообщение об ошибке
    printk("Connection object could not be inserted %p\n", conn);
}


// Функция, которая проверяет, есть ли свободное место в массиве conn_mode
static bool is_conn_slot_free(void)
{

    // Перебираем массив conn_mode и проверяем, есть ли свободное место
    for (size_t i = 0; i < CONFIG_BT_HIDS_MAX_CLIENT_COUNT; i++) {
        // Если найдено свободное место, возвращаем true
        if (!conn_mode[i].conn) {
            return true;
        }
    }

    // Если не найдено свободного места, возвращаем false
    return false;
}


// Функция, которая вызывается при установлении соединения
static void connected(struct bt_conn *conn, uint8_t err)
{
    // Буфер для адреса устройства
    char addr[BT_ADDR_LE_STR_LEN];

    // Останавливаем рекламные сообщения
    is_adv_running = false;

    // Преобразуем адрес устройства в строку
    bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

    // Если произошла ошибка, выводим сообщение об ошибке
    if (err) {
        // Если ошибка связана с таймаутом рекламных сообщений, выводим сообщение и запускаем рекламные сообщения снова
        if (err == BT_HCI_ERR_ADV_TIMEOUT) {
            printk("Direct advertising to %s timed out\n", addr);
            k_work_submit(&adv_work);
        } else {

            // Если ошибка не связана с таймаутом, выводим сообщение об ошибке
            printk("Failed to connect to %s (%u)\n", addr, err);
        }
        return;
    }

    // Выводим сообщение о соединении
    printk("Connected %s\n", addr);

    // Уведомляем сервис HID о соединении
    err = bt_hids_connected(&hids_obj, conn);

    // Если произошла ошибка, выводим сообщение об ошибке
    if (err) {
        printk("Failed to notify HID service about connection\n");
        return;
    }

    // Добавляем объект соединения в массив
    insert_conn_object(conn);

    // Если есть свободное место в массиве, запускаем рекламные сообщения снова
    if (is_conn_slot_free()) {
        advertising_start();
    }
}


// Функция, которая вызывается при разрыве соединения
static void disconnected(struct bt_conn *conn, uint8_t reason)
{
    // Код ошибки
    int err;


    // Буфер для адреса устройства
    char addr[BT_ADDR_LE_STR_LEN];

    // Преобразуем адрес устройства в строку
    bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

    // Выводим сообщение о разрыве соединения
    printk("Disconnected from %s (reason %u)\n", addr, reason);

    // Уведомляем сервис HID о разрыве соединения
    err = bt_hids_disconnected(&hids_obj, conn);

    // Если произошла ошибка, выводим сообщение об ошибке
    if (err) {
        printk("Failed to notify HID service about disconnection\n");
    }

    // Находим и удаляем объект соединения из массива
    for (size_t i = 0; i < CONFIG_BT_HIDS_MAX_CLIENT_COUNT; i++) {
        if (conn_mode[i].conn == conn) {
            conn_mode[i].conn = NULL;
            break;
        }
    }

    // Запускаем рекламные сообщения снова
    advertising_start();
}



#ifdef CONFIG_BT_HIDS_SECURITY_ENABLED
// Функция, которая вызывается при изменении уровня безопасности соединения
static void security_changed(struct bt_conn *conn, bt_security_t level,
        enum bt_security_err err)
{
    // Буфер для адреса устройства
    char addr[BT_ADDR_LE_STR_LEN];

    // Преобразуем адрес устройства в строку
    bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

    // Если не произошла ошибка, выводим сообщение об изменении уровня безопасности
    if (!err) {
        printk("Security changed: %s level %u\n", addr, level);
    } else {
        // Если произошла ошибка, выводим сообщение об ошибке безопасности
        printk("Security failed: %s level %u err %d\n", addr, level, err);
    }
}
#endif


BT_CONN_CB_DEFINE(conn_callbacks) = {
	.connected = connected,
	.disconnected = disconnected,
#ifdef CONFIG_BT_HIDS_SECURITY_ENABLED
	.security_changed = security_changed,
#endif
};



// Функция, которая обрабатывает события управления питанием HID
static void hids_pm_evt_handler(enum bt_hids_pm_evt evt,
        struct bt_conn *conn)
{
    // Буфер для адреса устройства
    char addr[BT_ADDR_LE_STR_LEN];

    // Индекс в массиве conn_mode
    size_t i;

    // Находим индекс в массиве conn_mode, соответствующий соединению
    for (i = 0; i < CONFIG_BT_HIDS_MAX_CLIENT_COUNT; i++) {
        if (conn_mode[i].conn == conn) {
            break;
        }
    }

    // Если не найден индекс, выходим
    if (i >= CONFIG_BT_HIDS_MAX_CLIENT_COUNT) {
        return;
    }

    // Преобразуем адрес устройства в строку
    bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

    // Обрабатываем событие управления питанием HID
    switch (evt) {
    case BT_HIDS_PM_EVT_BOOT_MODE_ENTERED:
        // Входим в режим загрузки
        printk("Boot mode entered %s\n", addr);
        conn_mode[i].in_boot_mode = true;
        break;

    case BT_HIDS_PM_EVT_REPORT_MODE_ENTERED:
        // Входим в режим отчета
        printk("Report mode entered %s\n", addr);
        conn_mode[i].in_boot_mode = false;
        break;

    default:
        // Неизвестное событие
        break;
    }
}


static void hid_init(void)
{
	int err;
	struct bt_hids_init_param hids_init_param = { 0 };
	struct bt_hids_inp_rep *hids_inp_rep;
	static const uint8_t mouse_movement_mask[DIV_ROUND_UP(INPUT_REP_MOVEMENT_LEN, 8)] = {0};

	static const uint8_t report_map[] = {
		0x05, 0x01,     /* Usage Page (Generic Desktop) */
		0x09, 0x02,     /* Usage (Mouse) */

		0xA1, 0x01,     /* Collection (Application) */

		/* Report ID 1: Mouse buttons + scroll/pan */
		0x85, 0x01,       /* Report Id 1 */
		0x09, 0x01,       /* Usage (Pointer) */
		0xA1, 0x00,       /* Collection (Physical) */
		0x95, 0x05,       /* Report Count (3) */
		0x75, 0x01,       /* Report Size (1) */
		0x05, 0x09,       /* Usage Page (Buttons) */
		0x19, 0x01,       /* Usage Minimum (01) */
		0x29, 0x05,       /* Usage Maximum (05) */
		0x15, 0x00,       /* Logical Minimum (0) */
		0x25, 0x01,       /* Logical Maximum (1) */
		0x81, 0x02,       /* Input (Data, Variable, Absolute) */
		0x95, 0x01,       /* Report Count (1) */
		0x75, 0x03,       /* Report Size (3) */
		0x81, 0x01,       /* Input (Constant) for padding */
		0x75, 0x08,       /* Report Size (8) */
		0x95, 0x01,       /* Report Count (1) */
		0x05, 0x01,       /* Usage Page (Generic Desktop) */
		0x09, 0x38,       /* Usage (Wheel) */
		0x15, 0x81,       /* Logical Minimum (-127) */
		0x25, 0x7F,       /* Logical Maximum (127) */
		0x81, 0x06,       /* Input (Data, Variable, Relative) */
		0x05, 0x0C,       /* Usage Page (Consumer) */
		0x0A, 0x38, 0x02, /* Usage (AC Pan) */
		0x95, 0x01,       /* Report Count (1) */
		0x81, 0x06,       /* Input (Data,Value,Relative,Bit Field) */
		0xC0,             /* End Collection (Physical) */


		/* Report ID 2: Mouse motion */
		0x85, 0x02,       /* Report Id 2 */
		0x09, 0x01,       /* Usage (Pointer) */
		0xA1, 0x00,       /* Collection (Physical) */
		0x75, 0x0C,       /* Report Size (12) */
		0x95, 0x02,       /* Report Count (2) */
		0x05, 0x01,       /* Usage Page (Generic Desktop) */
		0x09, 0x30,       /* Usage (X) */
		0x09, 0x31,       /* Usage (Y) */
		0x16, 0x01, 0xF8, /* Logical maximum (2047) */
		0x26, 0xFF, 0x07, /* Logical minimum (-2047) */
		0x81, 0x06,       /* Input (Data, Variable, Relative) */
		0xC0,             /* End Collection (Physical) */
		0xC0,             /* End Collection (Application) */

		/* Report ID 3: Advanced buttons */
		0x05, 0x0C,       /* Usage Page (Consumer) */
		0x09, 0x01,       /* Usage (Consumer Control) */
		0xA1, 0x01,       /* Collection (Application) */
		0x85, 0x03,       /* Report Id (3) */
		0x15, 0x00,       /* Logical minimum (0) */
		0x25, 0x01,       /* Logical maximum (1) */
		0x75, 0x01,       /* Report Size (1) */
		0x95, 0x01,       /* Report Count (1) */

		0x09, 0xCD,       /* Usage (Play/Pause) */
		0x81, 0x06,       /* Input (Data,Value,Relative,Bit Field) */
		0x0A, 0x83, 0x01, /* Usage (Consumer Control Configuration) */
		0x81, 0x06,       /* Input (Data,Value,Relative,Bit Field) */
		0x09, 0xB5,       /* Usage (Scan Next Track) */
		0x81, 0x06,       /* Input (Data,Value,Relative,Bit Field) */
		0x09, 0xB6,       /* Usage (Scan Previous Track) */
		0x81, 0x06,       /* Input (Data,Value,Relative,Bit Field) */

		0x09, 0xEA,       /* Usage (Volume Down) */
		0x81, 0x06,       /* Input (Data,Value,Relative,Bit Field) */
		0x09, 0xE9,       /* Usage (Volume Up) */
		0x81, 0x06,       /* Input (Data,Value,Relative,Bit Field) */
		0x0A, 0x25, 0x02, /* Usage (AC Forward) */
		0x81, 0x06,       /* Input (Data,Value,Relative,Bit Field) */
		0x0A, 0x24, 0x02, /* Usage (AC Back) */
		0x81, 0x06,       /* Input (Data,Value,Relative,Bit Field) */
		0xC0              /* End Collection */
	};


	// Установка карты отчетов для HID-устройства
	hids_init_param.rep_map.data = report_map;
	hids_init_param.rep_map.size = sizeof(report_map);

	// Установка информации о HID-устройстве
	hids_init_param.info.bcd_hid = BASE_USB_HID_SPEC_VERSION; // Версия спецификации HID
	hids_init_param.info.b_country_code = 0x00; // Код страны
	hids_init_param.info.flags = (BT_HIDS_REMOTE_WAKE | // Флаги HID-устройства
								BT_HIDS_NORMALLY_CONNECTABLE);


	// Установка входных отчетов для HID-устройства
	hids_inp_rep = &hids_init_param.inp_rep_group_init.reports[0];
	hids_inp_rep->size = INPUT_REP_BUTTONS_LEN; // Размер отчета о кнопках
	hids_inp_rep->id = INPUT_REP_REF_BUTTONS_ID; // Идентификатор отчета о кнопках
	hids_init_param.inp_rep_group_init.cnt++; // Увеличение счетчика входных отчетов


	// Установка следующего входного отчета
	hids_inp_rep++;
	hids_inp_rep->size = INPUT_REP_MOVEMENT_LEN; // Размер отчета о движении
	hids_inp_rep->id = INPUT_REP_REF_MOVEMENT_ID; // Идентификатор отчета о движении
	hids_inp_rep->rep_mask = mouse_movement_mask; // Маска отчета о движении
	hids_init_param.inp_rep_group_init.cnt++; // Увеличение счетчика входных отчетов


	// Установка следующего входного отчета
	hids_inp_rep++;
	hids_inp_rep->size = INPUT_REP_MEDIA_PLAYER_LEN; // Размер отчета о медиаплеере
	hids_inp_rep->id = INPUT_REP_REF_MPLAYER_ID; // Идентификатор отчета о медиаплеере
	hids_init_param.inp_rep_group_init.cnt++; // Увеличение счетчика входных отчетов


	// Установка флага мыши и обработчика событий
	hids_init_param.is_mouse = true; // Флаг мыши
	hids_init_param.pm_evt_handler = hids_pm_evt_handler; // Обработчик событий


	// Инициализация HID-устройства
	err = bt_hids_init(&hids_obj, &hids_init_param);
	__ASSERT(err == 0, "HIDS initialization failed\n"); // Проверка ошибки инициализации
}



static void mouse_movement_send(int16_t x_delta, int16_t y_delta)
{
  // Цикл по всем клиентам
  for (size_t i = 0; i < CONFIG_BT_HIDS_MAX_CLIENT_COUNT; i++) {
    // Если клиент не подключен, пропускаем его
    if (!conn_mode[i].conn) {
      continue;
    }


    // Если клиент находится в режиме загрузки, отправляем отчет в формате загрузки
    if (conn_mode[i].in_boot_mode) {
      // Ограничиваем значения x и y в диапазоне [-128, 127]
      x_delta = MAX(MIN(x_delta, SCHAR_MAX), SCHAR_MIN);
      y_delta = MAX(MIN(y_delta, SCHAR_MAX), SCHAR_MIN);

      // Отправляем отчет о движении мыши в формате загрузки
      bt_hids_boot_mouse_inp_rep_send(&hids_obj,
                                      conn_mode[i].conn,
                                      NULL,
                                      (int8_t) x_delta,
                                      (int8_t) y_delta,
                                      NULL);
    } else {
      // Если клиент не находится в режиме загрузки, отправляем отчет в формате HID
      uint8_t x_buff[2];
      uint8_t y_buff[2];
      uint8_t buffer[INPUT_REP_MOVEMENT_LEN];


      // Ограничиваем значения x и y в диапазоне [-2047, 2047]
      int16_t x = MAX(MIN(x_delta, 0x07ff), -0x07ff);
      int16_t y = MAX(MIN(y_delta, 0x07ff), -0x07ff);

      // Преобразуем значения x и y в формат little-endian
      sys_put_le16(x, x_buff);
      sys_put_le16(y, y_buff);

      // Кодирование отчета
      BUILD_ASSERT(sizeof(buffer) == 3,
                   "Only 2 axis, 12-bit each, are supported");

      buffer[0] = x_buff[0];
      buffer[1] = (y_buff[0] << 4) | (x_buff[1] & 0x0f);
      buffer[2] = (y_buff[1] << 4) | (y_buff[0] >> 4);


      // Отправляем отчет о движении мыши в формате HID
      bt_hids_inp_rep_send(&hids_obj, conn_mode[i].conn,
                          INPUT_REP_MOVEMENT_INDEX,
                          buffer, sizeof(buffer), NULL);
    }
  }
}



static void mouse_handler(struct k_work *work)
{
  // Структура для хранения позиции мыши
  struct mouse_pos pos;

  // Цикл, который обрабатывает сообщения из очереди HID
  while (!k_msgq_get(&hids_queue, &pos, K_NO_WAIT)) {
    // Отправляет отчет о движении мыши на основе позиции мыши
    mouse_movement_send(pos.x_val, pos.y_val);
  }

}

#if defined(CONFIG_BT_HIDS_SECURITY_ENABLED)
static void auth_passkey_display(struct bt_conn *conn, unsigned int passkey)
{
  // Буфер для хранения адреса устройства
  char addr[BT_ADDR_LE_STR_LEN];
  // Преобразование адреса устройства в строку
  bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr)); 

  // Вывод сообщения с проходным кодом
  printk("Passkey for %s: %06u\n", addr, passkey);
}

static void auth_passkey_confirm(struct bt_conn *conn, unsigned int passkey)
{
  // Код ошибки
  int err;

  // Структура для хранения данных пары
  struct pairing_data_mitm pairing_data;

  // Инициализация структуры данных пары
  pairing_data.conn = bt_conn_ref(conn);
  pairing_data.passkey = passkey;

  // Отправка данных пары в очередь
  err = k_msgq_put(&mitm_queue, &pairing_data, K_NO_WAIT);
  if (err) {
    // Если очередь полна, выводим сообщение об ошибке
    printk("Pairing queue is full. Purge previous data.\n");
  }

  // Если это первая пара в очереди, запускаем обработку пары
  if (k_msgq_num_used_get(&mitm_queue) == 1) {
    k_work_submit(&pairing_work);
  }
}


static void auth_cancel(struct bt_conn *conn)
{
  // Буфер для хранения адреса устройства
  char addr[BT_ADDR_LE_STR_LEN];

  // Преобразование адреса устройства в строку
  bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

  // Вывод сообщения об отмене пары
  printk("Pairing cancelled: %s\n", addr);
}


static void pairing_complete(struct bt_conn *conn, bool bonded)
{
  // Буфер для хранения адреса устройства
  char addr[BT_ADDR_LE_STR_LEN];

  // Преобразование адреса устройства в строку
  bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

  // Вывод сообщения о завершении пары
  printk("Pairing completed: %s, bonded: %d\n", addr, bonded);
}


static void pairing_failed(struct bt_conn *conn, enum bt_security_err reason)
{
  // Буфер для хранения адреса устройства
  char addr[BT_ADDR_LE_STR_LEN];

  // Структура для хранения данных пары
  struct pairing_data_mitm pairing_data;

  // Проверка, есть ли данные пары в очереди
  if (k_msgq_peek(&mitm_queue, &pairing_data) != 0) {
    // Если данных нет, выходим из функции
    return;
  }

  // Проверка, соответствует ли соединение в очереди текущему соединению
  if (pairing_data.conn == conn) {
    // Если соответствует, освобождаем ресурс соединения и удаляем данные из очереди
    bt_conn_unref(pairing_data.conn);
    k_msgq_get(&mitm_queue, &pairing_data, K_NO_WAIT);
  }

  // Преобразование адреса устройства в строку
  bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

  // Вывод сообщения о неудаче пары
  printk("Pairing failed conn: %s, reason %d\n", addr, reason);
}


// Определение структуры conn_auth_callbacks для обработки аутентификации соединения
static struct bt_conn_auth_cb conn_auth_callbacks = {
    // Функция, которая отображает проходной код для аутентификации
    .passkey_display = auth_passkey_display,
    // Функция, которая подтверждает проходной код для аутентификации
    .passkey_confirm = auth_passkey_confirm,
    // Функция, которая обрабатывает отмену процесса аутентификации
    .cancel = auth_cancel,
};

// Определение структуры conn_auth_info_callbacks для обработки информации о паре соединения
static struct bt_conn_auth_info_cb conn_auth_info_callbacks = {
    // Функция, которая обрабатывает завершение процесса пары
    .pairing_complete = pairing_complete,
    // Функция, которая обрабатывает неудачу процесса пары
    .pairing_failed = pairing_failed
};
#else
static struct bt_conn_auth_cb conn_auth_callbacks;
static struct bt_conn_auth_info_cb conn_auth_info_callbacks;
#endif /* defined(CONFIG_BT_HIDS_SECURITY_ENABLED) */


// Функция, которая отправляет ответ на запрос числового сравнения для пары соединения
static void num_comp_reply(bool accept)
{
    // Структура для хранения данных пары
    struct pairing_data_mitm pairing_data;
    // Указатель на соединение
    struct bt_conn *conn;

    // Получение данных пары из очереди
    if (k_msgq_get(&mitm_queue, &pairing_data, K_NO_WAIT) != 0) {
        // Если данные не получены, выходим из функции
        return;
    }

    // Получение соединения из данных пары
    conn = pairing_data.conn;

    // Если ответ положительный, подтверждаем числовое сравнение
    if (accept) {
        // Подтверждаем числовое сравнение для соединения
        bt_conn_auth_passkey_confirm(conn);
        // Выводим сообщение о подтверждении числового сравнения
        printk("Numeric Match, conn %p\n", conn);
    } else {
        // Если ответ отрицательный, отменяем числовое сравнение
        bt_conn_auth_cancel(conn);
        // Выводим сообщение об отмене числового сравнения
        printk("Numeric Reject, conn %p\n", conn);
    }

    // Освобождаем ресурс соединения
    bt_conn_unref(pairing_data.conn);

    // Если в очереди есть еще данные, запускаем обработку следующей пары
    if (k_msgq_num_used_get(&mitm_queue)) {
        k_work_submit(&pairing_work);
    }
}


// Функция button_changed вызывается при изменении состояния кнопок
void button_changed(uint32_t button_state, uint32_t has_changed)
{
    bool data_to_send = false;
    struct mouse_pos pos;
    uint32_t buttons = button_state & has_changed;


    memset(&pos, 0, sizeof(struct mouse_pos));


    if (IS_ENABLED(CONFIG_BT_HIDS_SECURITY_ENABLED)) {
        if (k_msgq_num_used_get(&mitm_queue)) {
            if (buttons & KEY_PAIRING_ACCEPT) {
                num_comp_reply(true);
                return;
            }

            if (buttons & KEY_PAIRING_REJECT) {
                num_comp_reply(false);
                return;
            }
        }
    }

    if (buttons & KEY_LEFT_MASK) {
        pos.x_val -= MOVEMENT_SPEED;
        printk("%s(): left\n", __func__);
        data_to_send = true;
    }
    if (buttons & KEY_UP_MASK) {
        pos.y_val -= MOVEMENT_SPEED;
        printk("%s(): up\n", __func__);
        data_to_send = true;
    }
    if (buttons & KEY_RIGHT_MASK) {
        pos.x_val += MOVEMENT_SPEED;
        printk("%s(): right\n", __func__);
        data_to_send = true;
    }
    if (buttons & KEY_DOWN_MASK) {
        pos.y_val += MOVEMENT_SPEED;
        printk("%s(): down\n", __func__);
        data_to_send = true;
    }


    if (data_to_send) {
        int err;
        err = k_msgq_put(&hids_queue, &pos, K_NO_WAIT);
        if (err) {
            printk("No space in the queue for button pressed\n");
            return;
        }
        if (k_msgq_num_used_get(&hids_queue) == 1) {
            k_work_submit(&hids_work);
        }
    }
}


// Функция, которая конфигурирует кнопки устройства
void configure_buttons(void)
{
    // Код ошибки
    int err;

    // Инициализация кнопок с функцией обратного вызова button_changed
    err = dk_buttons_init(button_changed);
    // Если инициализация не удалась, выводим сообщение об ошибке
    if (err) {
        printk("Cannot init buttons (err: %d)\n", err);
    }
}


// Функция, которая уведомляет о текущем уровне заряда батареи
static void bas_notify(void)
{
    // Текущий уровень заряда батареи
    uint8_t battery_level = bt_bas_get_battery_level();

    // Уменьшаем уровень заряда батареи на 1%
    battery_level--;

    // Если уровень заряда батареи достигнул 0%, устанавливаем его в 100%
    if (!battery_level) {
        battery_level = 100U;
    }

    // Устанавливаем новый уровень заряда батареи
    bt_bas_set_battery_level(battery_level);
}


// Основная функция программы
int main(void)
{
    // Код ошибки
    int err;

    // Выводим сообщение о начале работы примера Bluetooth Peripheral HIDS mouse
    printk("Starting Bluetooth Peripheral HIDS mouse example\n");

    // Если включена безопасность HIDS, регистрируем функции обратного вызова для авторизации
    if (IS_ENABLED(CONFIG_BT_HIDS_SECURITY_ENABLED)) {
        // Регистрируем функции обратного вызова для авторизации
        err = bt_conn_auth_cb_register(&conn_auth_callbacks);
        if (err) {
            // Если регистрация не удалась, выводим сообщение об ошибке и завершаем работу
            printk("Failed to register authorization callbacks.\n");
            return 0;
        }

        // Регистрируем функции обратного вызова для авторизации информации
        err = bt_conn_auth_info_cb_register(&conn_auth_info_callbacks);
        if (err) {
            // Если регистрация не удалась, выводим сообщение об ошибке и завершаем работу
            printk("Failed to register authorization info callbacks.\n");
            return 0;
        }
    }

    // Инициализируем HID
    hid_init();

    // Включаем Bluetooth
    err = bt_enable(NULL);
    if (err) {
        // Если включение не удалось, выводим сообщение об ошибке и завершаем работу
        printk("Bluetooth init failed (err %d)\n", err);
        return 0;
    }

    // Выводим сообщение о успешном включении Bluetooth
    printk("Bluetooth initialized\n");

    // Инициализируем задачи для обработки событий мыши и рекламных сообщений
    k_work_init(&hids_work, mouse_handler);
    k_work_init(&adv_work, advertising_process);
    if (IS_ENABLED(CONFIG_BT_HIDS_SECURITY_ENABLED)) {
        // Если включена безопасность HIDS, инициализируем задачу для обработки пары
        k_work_init(&pairing_work, pairing_process);
    }

    // Если включены настройки, загружаем их
    if (IS_ENABLED(CONFIG_SETTINGS)) {
        settings_load();
    }

    // Начинаем рекламные сообщения
    advertising_start();

    // Конфигурируем кнопки
    configure_buttons();

    // Основной цикл программы
    while (1) {
        // Сон на 1 секунду
        k_sleep(K_SECONDS(1));
        // Симулируем изменение уровня заряда батареи
        bas_notify();
    }
}

void test_run_cmd1(int number)
{
    printk("Running test\n");
    struct mouse_pos pos;
    int err;
    
    memset(&pos, 0, sizeof(struct mouse_pos));

    pos.x_val -= MOVEMENT_SPEED;

    err = k_msgq_put(&hids_queue, &pos, K_NO_WAIT);
        if (err) {
            printk("No space in the queue for button pressed\n");
            return;
        }
        if (k_msgq_num_used_get(&hids_queue) == 1) {
            k_work_submit(&hids_work);
        }
}

void test_run_cmd2(int number)
{
    printk("Running test\n");
    struct mouse_pos pos;
    int err;
    
    memset(&pos, 0, sizeof(struct mouse_pos));

    pos.y_val -= MOVEMENT_SPEED;

    err = k_msgq_put(&hids_queue, &pos, K_NO_WAIT);
        if (err) {
            printk("No space in the queue for button pressed\n");
            return;
        }
        if (k_msgq_num_used_get(&hids_queue) == 1) {
            k_work_submit(&hids_work);
        }
}

void test_run_cmd3(int number)
{
    printk("Running test\n");
    struct mouse_pos pos;
    int err;
    memset(&pos, 0, sizeof(struct mouse_pos));
    pos.x_val += MOVEMENT_SPEED;
    err = k_msgq_put(&hids_queue, &pos, K_NO_WAIT);
        if (err) {
            printk("No space in the queue for button pressed\n");
            return;
        }
        if (k_msgq_num_used_get(&hids_queue) == 1) {
            k_work_submit(&hids_work);
        }
}

void test_run_cmd4(int number)
{
    printk("Running test\n");
    struct mouse_pos pos;
    int err;

    memset(&pos, 0, sizeof(struct mouse_pos));
    pos.y_val += MOVEMENT_SPEED;

    err = k_msgq_put(&hids_queue, &pos, K_NO_WAIT);
        if (err) {
            printk("No space in the queue for button pressed\n");
            return;
        }
        if (k_msgq_num_used_get(&hids_queue) == 1) {
            k_work_submit(&hids_work);
        }
}

void test_run_off(int number)
{
    printk("Running test\n");
    struct mouse_pos pos;
    int err;

    memset(&pos, 0, sizeof(struct mouse_pos));
    pos.y_val = 0;
    pos.x_val = 0;

    err = k_msgq_put(&hids_queue, &pos, K_NO_WAIT);
        if (err) {
            printk("No space in the queue for button pressed\n");
            return;
        }
        if (k_msgq_num_used_get(&hids_queue) == 1) {
            k_work_submit(&hids_work);
        }
}

SHELL_CMD_REGISTER(run1, NULL, "Run the test", test_run_cmd1);
SHELL_CMD_REGISTER(run2, NULL, "Run the test", test_run_cmd2);
SHELL_CMD_REGISTER(run3, NULL, "Run the test", test_run_cmd3);
SHELL_CMD_REGISTER(run4, NULL, "Run the test", test_run_cmd4);
SHELL_CMD_REGISTER(off, NULL, "Run the test", test_run_off);

