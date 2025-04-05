# CAD модель робота-черепахи
Модель робота хранится в проекте T-Flex CAD 17 (российский САПР - доступный по студенческой лицензии всем). Для изготовления робота потребуется дополнительно приобрести компоненты, которые указаны ниже.

![image](https://github.com/user-attachments/assets/1b902d44-8ded-4fbf-a194-8cbae08abcfe)

## Структура сборки (проекта)
Каждый файл содержит 3 слоя: основной, внешний и референс. Основной содержит элементы, которые нужно печатать, внешний - покупные компоненты, референс - используется для построения.
Для получения файла на печать в 3D-принтере, зайдите в меню "Файл" и выберите меню "Экспорт". В разделе "Полигональная графика" найдите удобный вам формат. Далее откроется меню выбора папки для экспорта, а также качество. Выбирайте максимальное, чтобы окружности не превратились в многоугольник с видимым кол-вом граней.
## Сборка робота
Для сборки робота, кроме распечатанных деталей - нужны покупные элементы электроники (о них ниже). Кроме этого, следует не забыть об установке вплавляемых гаек в нужные отверстия деталей.
Сборка происходит снизу-вверх. Сначала требуется установить электронику (здесь можно использовать скотч двухсторонний, либо любой клей по пластику), двигатели на своё место. К задней панели можно прикрутить держатель аккумуляторов. К передней же панели установить в отверстия камеру ESP32-Cam. Далее уже приступаем к установке элементов верхней крышки: лидар, IMU, индикатор заряда, зарядное гнездо и тумблер. После установки комплектующих следует заняться пайкай, но перед этим не забудьте установить третье шариковое колесо, а уже после пайки и подключения - прикрутите колёса.
## Схема подключения

[Ссылка на схему подключения](https://app.cirkitdesigner.com/project/3918250a-88dc-40dc-bfbd-3c72aa6bc115)

Важные моменты:
1. Проверяйте работу компонентов отдельно, это может сэкономить время.
2. Подстроечный резистор DC-DC конвертера настраивайте на 5 вольт заранее, путём вращения регулятора и замера напряжения.
3. На модуле индикации заряда не забудьте соединить перемычку 2S.

## Список комплектующих к покупках
Далее идёт таблица с комплектующими для покупки (как пример, будут указаны ссылки на товары в маркетплейсах). Итоговая стоимость составляет 10-15 тысяч российский рублей (в зависимости от стоимости доставки и места покупки).

| Название | Кол-во | Ссылка |
| ------------ | ------------ | ------------ |
| BMS 2S 20A плата защиты с балансировкой, 8.4 В | 1 | [Ozon](https://www.ozon.ru/product/bms-2s-20a-plata-zashchity-s-balansirovkoy-8-4-v-dlya-sborok-akkumulyatorov-1162983848) |
| Плата управления двигателем L298N | 1 | [Ozon](https://www.ozon.ru/product/arduino-plata-upravleniya-shagovym-dvigatelem-motorami-l298n-802306681) |
| ESP32 Wroom-32 30pin USB CH340C + плата расширения ESP32 | 1 | [Ozon](https://www.ozon.ru/product/esp32-wroom-32-30pin-usb-ch340c-wi-fi-bluetooth-plata-razrabotki-esp32-162562274) |
| ESP32-CAM с камерой | 1 | [Ozon](https://www.ozon.ru/product/modul-esp32-cam-s-kameroy-1680169828) |
| Понижающий DC-DC преобразователь напряжения LM2596S | 1 | [Ozon](https://www.ozon.ru/product/ponizhayushchiy-dc-dc-preobrazovatel-napryazheniya-lm2596s-1420457935) |
| Лазерный датчик LDS Lidar для робота-пылесоса Dreame D9 F9 D9 Pro D9 Plus L10 Pro W10 | 1 | [AliExpress](https://aliexpress.ru/item/1005007149385720.html) |
| АО "Энергия" Аккумуляторная батарейка 18650, 3.6 В, 3000 мАч | 2 | [Ozon](https://www.ozon.ru/product/ao-energiya-akkumulyatornaya-batareyka-18650-3-6-v-3000-mach-1-sht-466340583) |
| Держатель батареи 18650 с проводами | 1 | [Ozon](https://www.ozon.ru/product/holder-derzhatel-batarei-18650-s-provodami-1011294470) |
| Редукторный двигатель JGA25-370 DC 6V с колесом | 2 | [AliExpress](https://aliexpress.ru/item/1005006399238256.html?spm=a2g2w.orderdetail.0.0.6c97f4aa6whAtF&sku_id=1200004100545172) |
| Вплавляемая гайка M3x4.2x5 | 50 | [Ozon](https://www.ozon.ru/product/gayka-m3-50-sht-1176501162) |
| Провод Arduino "мама-папа", 20 см | 20 | [Ozon](https://www.ozon.ru/product/provoda-peremychki-dupont-mama-papa-20-sm-20-shtuk-dlya-arduino-stm32-nodemcu-raspberry-pi-1335588425) |
| Провод многожильный 24AWG (0.2 мм2) в силиконовой изоляции. Луженая медь. 5 цветов по 10 метров. | 1 | [Ozon](https://www.ozon.ru/product/provod-mnogozhilnyy-24awg-0-2-mm2-v-silikonovoy-izolyatsii-luzhenaya-med-5-tsvetov-po-10-metrov-1352478338) |
| Гнездо питания DC-022 под разъем штырьковый (5.5x2.5 мм) | 1 | [Ozon](https://www.ozon.ru/product/gnezdo-pitaniya-dc-022-pod-razem-shtyrkovyy-5-5x2-5-mm-komplekt-2-sht-117336582) |
| Модуль пространственной ориентации IMU GY-91 MPU9250 | 1 | [Ozon](https://www.ozon.ru/product/modul-mpu9250-giroskop-akselerometr) |
| Универсальный индикатор заряда 1-8S 4.2-33.6V | 1 | [Ozon](https://www.ozon.ru/product/bat-cap-indicator-1-8s-4-2-33-6v-blue-universalnyy-indikator-zaryada-napryazheniya-emkosti-dlya-828567663/?avtc=1&avte=2&avts=1724011911) |
| Сетевое зарядное устройство для Li-Ion батарей 8.4V 2A 2S | 1 | [Ozon](https://www.ozon.ru/product/setevoe-zaryadnoe-ustroystvo-dlya-li-ion-batarey-8-4v-2a-2s-285109631) |
| Кнопка включения | 1 | [Ozon](https://www.ozon.ru/product/tumbler-vyklyuchatel-gsmin-kcd11-on-off-3a-250v-ac-2pin-15x10-chernyy-1365221265/) |
| Шарик металлический | 1 | [Ozon](https://www.ozon.ru/product/sharik-stalnoy-dlya-rogatok-vystrel-10-mm-banka-50-sht-762581687/) |
| Винты DIN912 M3x8 | 50 | [Ozon](https://www.ozon.ru/product/dometizov-vint-m3-x-3-x-8-mm-golovka-tsilindricheskaya-50-sht-809130936) |