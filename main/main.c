#include <stdio.h> // Funciones de entrada/salida estándar
#include <string.h> // Funciones de manipulación de cadenas y memoria
#include <math.h> // Funciones matemáticas (sqrt, sin, cos, atan2)
#include <stdlib.h> // Funciones de utilidad general (atoi, atof)
#include "freertos/FreeRTOS.h" // Núcleo de sistema operativo FreeRTOS
#include "freertos/task.h" // Tareas y delays
#include "freertos/queue.h" // Colas para comunicación entre tareas
#include "driver/uart.h" // Control de puerto serie (GPS A7670SA-FASE)
#include "driver/gpio.h" // Control de pines GPIO
#include "driver/spi_master.h" // Control de bus SPI (ILI9341, tarjeta SD)
#include "sdmmc_cmd.h" // Comandos para tarjeta SD/MMC
#include "esp_vfs_fat.h" // Sistema de archivos FAT para SD
#include "esp_system.h" // Funciones del sistema ESP32
#include "esp_log.h" // Sistema de logging para depuración
#include "TFT_eSPI.h" // Librería para pantalla ILI9341

// PLACA Y MÓDULOS
// ESPWROOM32 XX5R69 ← ILI9341: 5V, GND, G14 (SCLK), G13 (MOSI), G2 (DC), G15 (CS), G26 (RST), G21 (BLK)
//                   ← A7670SA-FASE (GPS): 3.3V, GND, G16 (TXD), G17 (RXD)
//                   ← TF Micro SD Card Module: 5V, GND, G12 (MISO), G13 (MOSI), G14 (SCLK), G15 (CS)

// VARIABLES GLOBALES DE CONFIGURACIÓN GPS
#define UART_GPS_NUM UART_NUM_2 // Puerto UART utilizado para módulo GPS
#define BUF_SIZE_GPS 1024 // Tamaño de buffer para recepción de datos GPS
#define GPS_BAUD_RATE 9600 // Velocidad en baudios del módulo GPS

// VARIABLES GLOBALES DE CONFIGURACIÓN SPI (PANTALLA Y TARJETA SD)
#define PIN_NUM_MISO GPIO_NUM_12 // Pin para Master In Slave Out (tarjeta SD)
#define PIN_NUM_MOSI GPIO_NUM_13 // Pin para Master Out Slave In (pantalla y SD)
#define PIN_NUM_CLK GPIO_NUM_14 // Pin para reloj SPI (pantalla y SD)
#define PIN_NUM_CS_SD GPIO_NUM_15 // Pin para chip select de tarjeta SD
#define PIN_NUM_CS_TFT GPIO_NUM_15 // Pin para chip select de pantalla (compartido)
#define PIN_NUM_DC_TFT GPIO_NUM_2 // Pin para data/command de pantalla
#define PIN_NUM_RST_TFT GPIO_NUM_26 // Pin para reset de pantalla
#define PIN_NUM_BLK_TFT GPIO_NUM_21 // Pin para backlight de pantalla

// VARIABLES GLOBALES DE CONFIGURACIÓN MAPA
#define TAMANO_TILE 256 // Tamaño en píxeles de cada imagen de mapa (256x256)
#define ZOOM_MUNDO 7 // Nivel de zoom para mapa mundial (equivalente a 1000km de altura)
#define ZOOM_CIUDAD 18 // Nivel de zoom para mapas de ciudad (equivalente a 300m de altura)
#define MATRIZ_IMAGENES 3 // Tamaño de matriz cuadrada (3x3 imágenes)
#define PUNTO_AZUL_RADIO 4 // Radio en píxeles del punto azul que indica posición
#define RUTA_BASE_MAPS "/sdcard/Maps" // Ruta base donde se almacenan los mapas en SD
#define RUTA_ARCHIVO_CIUDADES "/sdcard/Cities.txt" // Ruta del archivo con coordenadas de ciudades

// VARIABLES GLOBALES DE COORDENADAS Y ESTADO
static double coordenada_latitud_actual = 0.0; // Latitud actual obtenida del GPS (grados decimales)
static double coordenada_longitud_actual = 0.0; // Longitud actual obtenida del GPS (grados decimales)
static char nombre_ciudad_actual[32] = "World"; // Nombre de ciudad actual o "World" para mapa mundial
static int gps_valido = 0; // Bandera que indica si coordenadas GPS son válidas (0 = inválido, 1 = válido)

// VARIABLES GLOBALES PARA MANEJO DE ARCHIVOS Y RUTAS
static char ruta_carpeta_actual[128] = ""; // Ruta completa de la carpeta actual de imágenes
static char nombres_archivos_matriz[9][64]; // Matriz 3x3 de nombres de archivos de imágenes (9 elementos)

// ESTRUCTURA PARA COORDENADAS GEOGRÁFICAS Y LÍMITES DE IMAGEN
typedef struct {
    double latitud_norte; // Límite norte de la imagen (coordenada máxima en latitud)
    double longitud_este; // Límite este de la imagen (coordenada máxima en longitud)
    double latitud_sur; // Límite sur de la imagen (coordenada mínima en latitud)
    double longitud_oeste; // Límite oeste de la imagen (coordenada mínima en longitud)
    double ratio_pixel_latitud; // Relación grados/píxel para latitud (cambio por pixel vertical)
    double ratio_pixel_longitud; // Relación grados/píxel para longitud (cambio por pixel horizontal)
} limites_imagen_t;

// VARIABLES GLOBALES PARA LÍMITES DE IMAGEN CENTRAL
static limites_imagen_t limites_imagen_central; // Límites geográficos de la imagen central en pantalla

// FUNCIONES
// Inicializa puerto UART para comunicación con módulo GPS A7670SA-FASE
static void inicializar_uart_gps(void)
{
    // Configuración de parámetros del puerto UART
    uart_config_t configuracion_uart = (
        .baud_rate = GPS_BAUD_RATE, // Velocidad de transmisión (9600 baudios)
        .data_bits = UART_DATA_8_BITS, // 8 bits de datos por byte
        .parity = UART_PARITY_DISABLE, // Sin bit de paridad
        .stop_bits = UART_STOP_BITS_1, // 1 bit de parada
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE, // Sin control de flujo por hardware
        .rx_flow_ctrl_thresh = 122, // Umbral para control de flujo (valor por defecto)
        .source_clk = UART_SCLK_DEFAULT // Reloj por defecto del sistema
    );

    uart_param_config(UART_GPS_NUM, &configuracion_uart); // Aplica configuración al puerto

    // Configura pines físicos para UART (GPIO16 = TX, GPIO17 = RX)
    uart_set_pin(UART_GPS_NUM, GPIO_NUM_16, GPIO_NUM_17, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);

    // Instala driver UART con buffer para recepción de datos
    uart_driver_install(UART_GPS_NUM, BUF_SIZE_GPS * 2, 0, 0, NULL, 0);
}

// Parsea sentencia NMEA GPRMC para extraer latitud y longitud
static int parsear_nmea_gprmc(const char *cadena_nmea, double *latitud, double *longitud)
{
    char copia_cadena[256]; // Copia local de la cadena NMEA para procesamiento seguro
    strncpy(copia_cadena, cadena_nmea, sizeof(copia_cadena) - 1);
    copia_cadena[sizeof(copia_cadena) - 1] = '\0'; // Asegura terminación nula

    // Verifica que la cadena comience con "$GPRMC" (formato válido)
    if (strncmp(copia_cadena, "$GPRMC", 6) != 0)
    {
        return 0; // No es sentencia GPRMC, retorna error
    }

    // Divide la cadena usando coma como separador de campos
    char *token = strtok(copia_cadena, ",");
    int indice_campo = 0; // Índice para contar campos procesados
    char campo_latitud[16] = ""; // Buffer para latitud en formato NMEA
    char hemisferio_latitud = 'N'; // Hemisferio norte (N) o sur (S)
    char campo_longitud[16] = ""; // Buffer para longitud en formato NMEA
    char hemisferio_longitud = 'E'; // Hemisferio este (E) o oeste (W)
    char estado_validez = 'V'; // Estado de validez (A = válido, V = inválido)

    // Procesa cada campo de la sentencia NMEA
    while (token != NULL && indice_campo < 12)
    {
        switch (indice_campo)
        {
            case 1: // Campo 1: Hora UTC (ignorado)
                break;
            
            case 2: // Campo 2: Estado de validez (A/V)
                if (strlen(token) > 0)
                {
                    estado_validez = token[0];
                }
                break;
            
            case 3: // Campo 3: Latitud en formato DDMM.MMMM
                strncpy(campo_latitud, token, sizeof(campo_latitud) - 1);
                break;
            
            case 4: // Campo 4: Hemisferio latitud (N/S)
                if (strlen(token) > 0)
                {
                    hemisferio_latitud = token[0];
                }
                break;
            
            case 5: // Campo 5: Longitud en formato DDDMM.MMMM
                strncpy(campo_longitud, token, sizeof(campo_longitud) - 1);
                break;
            
            case 6: // Campo 6: Hemisferio longitud (E/W)
                if (strlen(token) > 0)
                {
                    hemisferio_longitud = token[0];
                }
                break;
        }

        token = strtok(NULL, ","); // Obtiene siguiente campo
        indice_campo++; // Incrementa contador de campos
    }

    // Verifica que datos sean válidos (estado 'A' = Active)
    if (estado_validez != 'A')
    {
        return 0; // Datos no válidos, retorna error
    }

    // CONVERSIÓN DE LATITUD NMEA A GRADOS DECIMALES
    if (strlen(campo_latitud) >= 4)
    {
        // Extrae grados (primeros 2 dígitos)
        char grados_str[3] = {0};
        strncpy(grados_str, campo_latitud, 2);
        double grados_lat = atof(grados_str);

        // Extrace minutos (resto de la cadena)
        double minutos_lat = atof(campo_latitud + 2);

        // Convierte a grados decimales: grados + (minutos/60)
        *latitud = grados_lat + (minutos_lat / 60.0);

        // Aplica signo según hemisferio (Sur = negativo)
        if (hemisferio_latitud == 'S')
        {
            *latitud = -(*latitud);
        }
    }
    else
    {
        return 0; // Formato de latitud inválido
    }

    // CONVERSIÓN DE LONGITUD NMEA A GRADOS DECIMALES
    if (strlen(campo_longitud) >= 5)
    {
        // Extrae grados (primeros 3 dígitos para longitud)
        char grados_str[4] = {0};
        strncpy(grados_str, campo_longitud, 3);
        double grados_lon = atof(grados_str);

        // Extrace minutos (resto de la cadena)
        double minutos_lon = atof(campo_longitud + 3);

        // Convierte a grados decimales: grados + (minutos/60)
        *longitud = grados_lon + (minutos_lon / 60.0);

        // Aplica signo según hemisferio (Oeste = negativo)
        if (hemisferio_longitud == 'W')
        {
            *longitud = -(*longitud);
        }
    }
    else
    {
        return 0; // Formato de longitud inválido
    }

    return 1; // Conversión exitosa
}

// Tarea que lee datos GPS continuamente y actualiza coordenadas globales
static void tarea_lectura_gps(void *parametros)
{
    uint8_t buffer_datos[BUF_SIZE_GPS]; // Buffer para almacenar datos recibidos del GPS
    char cadena_nmea_completa[256]; // Buffer para acumular cadena NMEA completa
    int indice_cadena = 0; // Índice para construir cadena carácter por carácter

    while (1)
    {
        // Lee datos disponibles del puerto UART (no bloqueante)
        int longitud_recibida = uart_read_bytes(UART_GPS_NUM, buffer_datos, sizeof(buffer_datos) - 1, 20 / portTICK_PERIOD_MS);

        if (longitud_recibida > 0)
        {
            // Procesa cada byte recibido
            for (int i = 0; i < longitud_recibida; i++)
            {
                char caracter_actual = buffer_datos[i];

                // Inicio de nueva sentencia NMEA (carácter '$')
                if (caracter_actual == '$')
                {
                    indice_cadena = 0; // Reinicia índice para nueva cadena
                    cadena_nmea_completa[indice_cadena++] = caracter_actual; // Almacena '$'
                }
                // Fin de sentencia NMEA (retorno de carro)
                else if (caracter_actual == '\r' || caracter_actual == '\n')
                {
                    if (indice_cadena > 0)
                    {
                        // Termina cadena con carácter nulo
                        cadena_nmea_completa[indice_cadena] = '\0';

                        // Intenta parsear sentencia GPRMC para obtener coordenadas
                        double latitud_temporal = 0.0;
                        double longitud_temporal = 0.0;

                        if (parsear_nmea_gprmc(cadena_nmea_completa, &latitud_temporal, &longitud_temporal))
                        {
                            // Actualiza coordenadas globales (protegido por mutex en aplicación real)
                            coordenada_latitud_actual = latitud_temporal;
                            coordenada_longitud_actual = longitud_temporal;
                            gps_valido = 1; // Marca coordenadas como válidas
                        }

                        indice_cadena = 0; // Prepara para siguiente sentencia
                    }
                }
                // Carácter normal de la sentencia (almacena si hay espacio)
                else if (indice_cadena < (int)sizeof(cadena_nmea_completa) - 1)
                {
                    cadena_nmea_completa[indice_cadena++] = caracter_actual;
                }
            }
        }

        vTaskDelay(50 / portTICK_PERIOD_MS); // Pequeño delay para no consumir CPU excesivamente
    }
}

// Monta sistema de archivos FAT en tarjeta SD
static int montar_sistema_archivos_sd(void)
{
    // Configuración de host SD/MMC
    sdmmc_host_t host_config = SDSPI_HOST_DEFAULT();
    host_config.slot = SPI2_HOST; // Utiliza SPI host 2

    // Configuración de bus SPI para tarjeta SD
    spi_bus_config_t bus_config = (
        .mosi_io_num = PIN_NUM_MOSI,
        .miso_io_num = PIN_NUM_MISO,
        .sclk_io_num = PIN_NUM_CLK,
        .quadwp_io_num = -1, // No usado en modo SPI
        .quadhd_io_num = -1, // No usado en modo SPI
        .max_transfer_sz = 4092 // Tamaño máximo de transferencia
    );

    // Inicializa bus SPI
    esp_err_t retorno_spi = spi_bus_initialize(host_config.slot, &bus_config, SDSPI_DEFAULT_DMA);
    
    if (retorno_spi != ESP_OK)
    {
        return 0; // Error inicializando bus SPI
    }

    // Configuración de slot SDSPI
    sdspi_device_config_t slot_config = SDSPI_DEVICE_CONFIG_DEFAULT();
    slot_config.gpio_cs = PIN_NUM_CS_SD; // Pin de chip select
    slot_config.host_id = host_config.slot; // ID del host SPI

    // Opciones para montar sistema de archivos FAT
    esp_vfs_fat_sdmmc_mount_config_t mount_config = (
        .format_if_mount_failed = false, // No formatear si falla montaje
        .max_files = 5, // Máximo número de archivos abiertos simultáneamente
        .allocation_unit_size = 16 * 1024 // Tamaño de unidad de asignación
    );

    // Variables para almacenar información de tarjeta
    sdmmc_card_t *tarjeta_sd;
    
    // Monta sistema de archivos FAT
    esp_err_t retorno_montaje = esp_vfs_fat_sdspi_mount("/sdcard", &host_config, &slot_config, &mount_config, &tarjeta_sd);

    if (retorno_montaje != ESP_OK)
    {
        return 0; // Error montando tarjeta SD
    }

    return 1; // Montaje exitoso
}

// Convierte coordenadas geográficas a coordenadas de tile en nivel de zoom específico
static void convertir_coordenadas_a_tile(double latitud, double longitud, int zoom, int *tile_x, int *tile_y)
{
    // Convierte latitud a radianes para cálculos trigonométricos
    double latitud_radianes = latitud * M_PI / 180.0;

    // Fórmula de conversión de Web Mercator para coordenada X (longitud)
    double numero_tiles = pow(2.0, zoom);
    *tile_x = (int)((longitud + 180.0) / 360.0 * numero_tiles);

    // Fórmula de conversión de Web Mercator para coordenada Y (latitud)
    double mercator_n = log(tan(latitud_radianes) + 1.0 / cos(latitud_radianes));
    *tile_y = (int)((1.0 - mercator_n / M_PI) / 2.0 * numero_tiles);
}

// Convierte coordenadas de tile a límites geográficos de la imagen
static limites_imagen_t convertir_tile_a_limites(int tile_x, int tile_y, int zoom)
{
    limites_imagen_t limites;
    
    // Calcula número total de tiles en este nivel de zoom
    double numero_tiles = pow(2.0, zoom);

    // Calcula longitud oeste (límite izquierdo)
    limites.longitud_oeste = (tile_x / numero_tiles * 360.0) - 180.0;

    // Calcula longitud este (límite derecho, siguiente tile)
    limites.longitud_este = ((tile_x + 1) / numero_tiles * 360.0) - 180.0;

    // Calcula latitud norte (límite superior) usando proyección Mercator inversa
    double n_norte = M_PI * (1.0 - 2.0 * tile_y / numero_tiles);
    limites.latitud_norte = 180.0 / M_PI * atan(sinh(n_norte));

    // Calcula latitud sur (límite inferior, siguiente tile en Y)
    double n_sur = M_PI * (1.0 - 2.0 * (tile_y + 1) / numero_tiles);
    limites.latitud_sur = 180.0 / M_PI * atan(sinh(n_sur));

    // Calcula relación grados/píxel para conversión precisa de coordenadas a posición en pantalla
    limites.ratio_pixel_latitud = (limites.latitud_norte - limites.latitud_sur) / TAMANO_TILE;
    limites.ratio_pixel_longitud = (limites.longitud_este - limites.longitud_oeste) / TAMANO_TILE;

    return limites;
}

// Genera nombre de archivo de imagen basado en límites geográficos
static void generar_nombre_archivo_imagen(limites_imagen_t limites, char *nombre_buffer, int tamano_buffer)
{
    // Convierte todas las coordenadas a valores absolutos (positivos)
    double lat_norte_abs = fabs(limites.latitud_norte);
    double lon_este_abs = fabs(limites.longitud_este);
    double lat_sur_abs = fabs(limites.latitud_sur);
    double lon_oeste_abs = fabs(limites.longitud_oeste);

    // Formatea cada coordenada con 4 decimales y elimina ceros innecesarios
    char norte_str[16], este_str[16], sur_str[16], oeste_str[16];
    
    snprintf(norte_str, sizeof(norte_str), "%.4f", lat_norte_abs);
    snprintf(este_str, sizeof(este_str), "%.4f", lon_este_abs);
    snprintf(sur_str, sizeof(sur_str), "%.4f", lat_sur_abs);
    snprintf(oeste_str, sizeof(oeste_str), "%.4f", lon_oeste_abs);

    // Elimina ceros finales después del punto decimal
    for (int i = 0; i < 4; i++)
    {
        char *cadenas[] = {norte_str, este_str, sur_str, oeste_str};
        
        for (int j = 0; j < 4; j++)
        {
            char *punto = strchr(cadenas[j], '.');
            
            if (punto)
            {
                // Elimina ceros finales
                char *fin = cadenas[j] + strlen(cadenas[j]) - 1;
                
                while (fin > punto && *fin == '0')
                {
                    *fin = '\0';
                    fin--;
                }
                
                // Elimina punto decimal si no quedan dígitos después
                if (*(fin) == '.')
                {
                    *fin = '\0';
                }
            }
        }
    }

    // Asegura que haya al menos un dígito después del punto
    if (strchr(norte_str, '.') == NULL && strlen(norte_str) > 0)
    {
        strcat(norte_str, ".0");
    }
    
    if (strchr(este_str, '.') == NULL && strlen(este_str) > 0)
    {
        strcat(este_str, ".0");
    }
    
    if (strchr(sur_str, '.') == NULL && strlen(sur_str) > 0)
    {
        strcat(sur_str, ".0");
    }
    
    if (strchr(oeste_str, '.') == NULL && strlen(oeste_str) > 0)
    {
        strcat(oeste_str, ".0");
    }

    // Construye nombre final en formato: N{lat_norte}_E{lon_este}_S{lat_sur}_O{lon_oeste}.jpg
    snprintf(nombre_buffer, tamano_buffer, "N%s_E%s_S%s_O%s.jpg", norte_str, este_str, sur_str, oeste_str);
}

// Busca ciudad correspondiente a coordenadas en archivo Cities.txt
static int buscar_ciudad_por_coordenadas(double latitud, double longitud, char *nombre_ciudad, int tamano_nombre)
{
    FILE *archivo_ciudades = fopen(RUTA_ARCHIVO_CIUDADES, "r");
    
    if (archivo_ciudades == NULL)
    {
        return 0; // No se pudo abrir archivo
    }

    char linea_actual[256];
    int ciudad_encontrada = 0;

    // Lee archivo línea por línea
    while (fgets(linea_actual, sizeof(linea_actual), archivo_ciudades))
    {
        // Elimina caracteres de nueva línea al final
        linea_actual[strcspn(linea_actual, "\r\n")] = '\0';

        // Divide línea por comas
        char *token = strtok(linea_actual, ",");
        
        if (token == NULL)
        {
            continue; // Línea vacía o inválida
        }

        // Primer token es nombre de ciudad
        char nombre_linea[64];
        strncpy(nombre_linea, token, sizeof(nombre_linea) - 1);
        nombre_linea[sizeof(nombre_linea) - 1] = '\0';

        // Siguientes tokens son coordenadas
        token = strtok(NULL, ",");
        
        if (token == NULL) continue;
        
        double norte = atof(token);

        token = strtok(NULL, ",");
        
        if (token == NULL) continue;
        
        double este = atof(token);

        token = strtok(NULL, ",");
        
        if (token == NULL) continue;
        
        double sur = atof(token);

        token = strtok(NULL, ",");
        
        if (token == NULL) continue;
        
        double oeste = atof(token);

        // Verifica si coordenadas están dentro de los límites de la ciudad
        if (latitud <= norte && latitud >= sur && longitud <= este && longitud >= oeste)
        {
            strncpy(nombre_ciudad, nombre_linea, tamano_nombre - 1);
            nombre_ciudad[tamano_nombre - 1] = '\0';
            
            ciudad_encontrada = 1;
            
            break; // Ciudad encontrada, termina búsqueda
        }
    }

    fclose(archivo_ciudades);
    
    return ciudad_encontrada;
}

// Determina carpeta correcta (World o ciudad específica) basado en coordenadas actuales
static void determinar_carpeta_actual(void)
{
    // Si no hay coordenadas GPS válidas, usa mapa mundial por defecto
    if (!gps_valido)
    {
        strcpy(nombre_ciudad_actual, "World");
        snprintf(ruta_carpeta_actual, sizeof(ruta_carpeta_actual), "%s/World", RUTA_BASE_MAPS);
        
        return;
    }

    // Busca ciudad correspondiente a coordenadas actuales
    char nombre_ciudad_temp[32];
    
    if (buscar_ciudad_por_coordenadas(coordenada_latitud_actual, coordenada_longitud_actual, nombre_ciudad_temp, sizeof(nombre_ciudad_temp)))
    {
        // Ciudad encontrada: actualiza variables globales
        strcpy(nombre_ciudad_actual, nombre_ciudad_temp);
        snprintf(ruta_carpeta_actual, sizeof(ruta_carpeta_actual), "%s/%s", RUTA_BASE_MAPS, nombre_ciudad_actual);
    }
    else
    {
        // No está en ninguna ciudad: usar mapa mundial
        strcpy(nombre_ciudad_actual, "World");
        snprintf(ruta_carpeta_actual, sizeof(ruta_carpeta_actual), "%s/World", RUTA_BASE_MAPS);
    }
}

// Encuentra imagen correspondiente a coordenadas en la carpeta actual
static int encontrar_imagen_por_coordenadas(double latitud, double longitud, limites_imagen_t *limites, char *nombre_imagen, int tamano_nombre)
{
    // Determina nivel de zoom según si estamos en ciudad o mundo
    int zoom_actual = (strcmp(nombre_ciudad_actual, "World") == 0) ? ZOOM_MUNDO : ZOOM_CIUDAD;

    // Convierte coordenadas a tile correspondiente
    int tile_x, tile_y;
    convertir_coordenadas_a_tile(latitud, longitud, zoom_actual, &tile_x, &tile_y);

    // Convierte tile a límites geográficos
    *limites = convertir_tile_a_limites(tile_x, tile_y, zoom_actual);

    // Genera nombre de archivo esperado
    generar_nombre_archivo_imagen(*limites, nombre_imagen, tamano_nombre);

    // Construye ruta completa del archivo
    char ruta_completa[256];
    snprintf(ruta_completa, sizeof(ruta_completa), "%s/%s", ruta_carpeta_actual, nombre_imagen);

    // Verifica si archivo existe
    FILE *archivo_verificacion = fopen(ruta_completa, "r");
    
    if (archivo_verificacion != NULL)
    {
        fclose(archivo_verificacion);
        return 1; // Imagen encontrada
    }

    return 0; // Imagen no encontrada
}

// Carga matriz 3x3 de imágenes alrededor de la imagen central
static void cargar_matriz_imagenes(double latitud_central, double longitud_central)
{
    // Determina nivel de zoom
    int zoom_actual = (strcmp(nombre_ciudad_actual, "World") == 0) ? ZOOM_MUNDO : ZOOM_CIUDAD;

    // Convierte coordenadas centrales a tile
    int tile_x_central, tile_y_central;
    convertir_coordenadas_a_tile(latitud_central, longitud_central, zoom_actual, &tile_x_central, &tile_y_central);

    // Calcula tiles para matriz 3x3 alrededor del central
    for (int desplazamiento_y = -1; desplazamiento_y <= 1; desplazamiento_y++)
    {
        for (int desplazamiento_x = -1; desplazamiento_x <= 1; desplazamiento_x++)
        {
            int tile_x = tile_x_central + desplazamiento_x;
            int tile_y = tile_y_central + desplazamiento_y;

            // Convierte tile a límites
            limites_imagen_t limites = convertir_tile_a_limites(tile_x, tile_y, zoom_actual);

            // Genera nombre de archivo
            int indice_matriz = (desplazamiento_y + 1) * 3 + (desplazamiento_x + 1);
            generar_nombre_archivo_imagen(limites, nombres_archivos_matriz[indice_matriz], sizeof(nombres_archivos_matriz[0]));
        }
    }
}

// Dibuja punto azul que indica posición actual en la imagen central
static void dibujar_punto_posicion(TFT_eSPI *pantalla_tft)
{
    // Calcula posición del punto dentro de la imagen central (256x256 píxeles)
    double offset_latitud = limites_imagen_central.latitud_norte - coordenada_latitud_actual;
    double offset_longitud = coordenada_longitud_actual - limites_imagen_central.longitud_oeste;

    int pixel_y = (int)(offset_latitud / limites_imagen_central.ratio_pixel_latitud);
    int pixel_x = (int)(offset_longitud / limites_imagen_central.ratio_pixel_longitud);

    // Asegura que coordenadas estén dentro de la imagen
    if (pixel_x < 0) pixel_x = 0;
    if (pixel_x >= TAMANO_TILE) pixel_x = TAMANO_TILE - 1;
    if (pixel_y < 0) pixel_y = 0;
    if (pixel_y >= TAMANO_TILE) pixel_y = TAMANO_TILE - 1;

    // Calcula posición en pantalla completa (768x768 píxeles para 3x3 imágenes de 256x256)
    int posicion_pantalla_x = 256 + pixel_x; // Columna central (imagen 1,1)
    int posicion_pantalla_y = 256 + pixel_y; // Fila central (imagen 1,1)

    // Dibuja punto azul como círculo relleno
    pantalla_tft->fillCircle(posicion_pantalla_x, posicion_pantalla_y, PUNTO_AZUL_RADIO, TFT_BLUE);
    
    // Dibuja borde blanco para mejor visibilidad
    pantalla_tft->drawCircle(posicion_pantalla_x, posicion_pantalla_y, PUNTO_AZUL_RADIO, TFT_WHITE);
}

// Inicializa pantalla ILI9341 con TFT_eSPI
static TFT_eSPI* inicializar_pantalla_tft(void)
{
    // Crea objeto de pantalla
    TFT_eSPI *pantalla_tft = new TFT_eSPI();

    // Inicializa pantalla
    pantalla_tft->init();
    
    // Configura rotación (0 = normal, 1 = 90°, 2 = 180°, 3 = 270°)
    pantalla_tft->setRotation(0);
    
    // Establece color de fondo negro
    pantalla_tft->fillScreen(TFT_BLACK);
    
    // Habilita backlight
    gpio_set_direction(PIN_NUM_BLK_TFT, GPIO_MODE_OUTPUT);
    gpio_set_level(PIN_NUM_BLK_TFT, 1);

    return pantalla_tft;
}

// Carga y muestra imagen JPEG en posición específica de la pantalla
static void mostrar_imagen_posicion(TFT_eSPI *pantalla_tft, const char *nombre_archivo, int posicion_x, int posicion_y)
{
    // Construye ruta completa del archivo
    char ruta_completa[256];
    snprintf(ruta_completa, sizeof(ruta_completa), "%s/%s", ruta_carpeta_actual, nombre_archivo);

    // Verifica si archivo existe
    FILE *archivo = fopen(ruta_completa, "rb");
    
    if (archivo == NULL)
    {
        // Dibuja rectángulo negro si imagen no existe
        pantalla_tft->fillRect(posicion_x, posicion_y, TAMANO_TILE, TAMANO_TILE, TFT_BLACK);
        
        return;
    }

    // Obtiene tamaño del archivo
    fseek(archivo, 0, SEEK_END);
    long tamano_archivo = ftell(archivo);
    fseek(archivo, 0, SEEK_SET);

    // Asigna memoria para datos de imagen
    uint8_t *buffer_imagen = (uint8_t *)malloc(tamano_archivo);
    
    if (buffer_imagen == NULL)
    {
        fclose(archivo);
        return;
    }

    // Lee archivo completo a memoria
    fread(buffer_imagen, 1, tamano_archivo, archivo);
    fclose(archivo);

    // Dibuja imagen JPEG en pantalla (TFT_eSPI tiene métodos para dibujar JPEG)
    pantalla_tft->drawJpg(buffer_imagen, tamano_archivo, posicion_x, posicion_y, TAMANO_TILE, TAMANO_TILE);

    // Libera memoria del buffer
    free(buffer_imagen);
}

// Actualiza pantalla completa con matriz 3x3 de imágenes y punto de posición
static void actualizar_pantalla_completa(TFT_eSPI *pantalla_tft)
{
    // Carga y muestra cada imagen de la matriz 3x3
    for (int fila = 0; fila < 3; fila++)
    {
        for (int columna = 0; columna < 3; columna++)
        {
            int indice_matriz = fila * 3 + columna;
            int posicion_x = columna * TAMANO_TILE;
            int posicion_y = fila * TAMANO_TILE;

            mostrar_imagen_posicion(pantalla_tft, nombres_archivos_matriz[indice_matriz], posicion_x, posicion_y);
        }
    }

    // Dibuja punto azul de posición actual
    if (gps_valido)
    {
        dibujar_punto_posicion(pantalla_tft);
    }
}

// Tarea principal que actualiza mapa según coordenadas GPS
static void tarea_actualizacion_mapa(void *parametros)
{
    // Inicializa pantalla
    TFT_eSPI *pantalla_tft = inicializar_pantalla_tft();

    // Variables para seguimiento de cambios
    double latitud_anterior = 0.0;
    double longitud_anterior = 0.0;
    char ciudad_anterior[32] = "";
    int primera_actualizacion = 1;

    while (1)
    {
        // Si no hay coordenadas GPS válidas, espera y reintenta
        if (!gps_valido)
        {
            vTaskDelay(1000 / portTICK_PERIOD_MS);
            
            continue;
        }

        // Verifica si hubo cambio significativo en coordenadas (0.0001 grados ≈ 11 metros)
        double cambio_latitud = fabs(coordenada_latitud_actual - latitud_anterior);
        double cambio_longitud = fabs(coordenada_longitud_actual - longitud_anterior);
        
        int cambio_ciudad = strcmp(nombre_ciudad_actual, ciudad_anterior) != 0;

        // Actualiza pantalla si hay cambio significativo o es primera actualización
        if (primera_actualizacion || cambio_ciudad || cambio_latitud > 0.0001 || cambio_longitud > 0.0001)
        {
            // Determina carpeta actual basada en coordenadas
            determinar_carpeta_actual();

            // Encuentra imagen central para coordenadas actuales
            char nombre_imagen_central[64];
            
            if (encontrar_imagen_por_coordenadas(coordenada_latitud_actual, coordenada_longitud_actual, &limites_imagen_central, nombre_imagen_central, sizeof(nombre_imagen_central)))
            {
                // Carga matriz 3x3 alrededor de imagen central
                cargar_matriz_imagenes(coordenada_latitud_actual, coordenada_longitud_actual);

                // Actualiza pantalla completa
                actualizar_pantalla_completa(pantalla_tft);

                // Actualiza variables de seguimiento
                latitud_anterior = coordenada_latitud_actual;
                longitud_anterior = coordenada_longitud_actual;
                strcpy(ciudad_anterior, nombre_ciudad_actual);
                primera_actualizacion = 0;
            }
        }

        vTaskDelay(500 / portTICK_PERIOD_MS); // Actualiza cada 500ms
    }
}

// PUNTO DE PARTIDA (FUNCIÓN PRINCIPAL)
void app_main(void)
{
    // Inicializa sistema de logging para depuración
    esp_log_level_set("*", ESP_LOG_INFO);

    // Monta sistema de archivos en tarjeta SD
    if (!montar_sistema_archivos_sd())
    {
        printf("Error mounting SD card\n");
        
        return;
    }

    // Inicializa puerto UART para GPS
    inicializar_uart_gps();

    // Crea tarea para lectura continua de GPS
    xTaskCreate(
        tarea_lectura_gps, // Función de la tarea
        "tarea_gps", // Nombre de la tarea
        4096, // Tamaño de pila en bytes
        NULL, // Parámetros pasados a la tarea
        3, // Prioridad (media)
        NULL // Manejador de tarea
    );

    // Crea tarea para actualización del mapa en pantalla
    xTaskCreate(
        tarea_actualizacion_mapa, // Función de la tarea
        "tarea_mapa", // Nombre de la tarea
        8192, // Tamaño de pila más grande (manejo de imágenes)
        NULL, // Parámetros pasados a la tarea
        4, // Prioridad (alta)
        NULL // Manejador de tarea
    );

    // Bucle principal mantiene programa en ejecución
    while (1)
    {
        vTaskDelay(1000 / portTICK_PERIOD_MS); // Espera 1 segundo
    }
}
