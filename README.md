# KL05Z_Libraries
Library for the Kinetis KL05Z microcontroller

# Project licensed with CC-BY - Attribution Alone 4.0 International Public License
https://creativecommons.org/licenses/by/4.0/


# Guia de Utilização 

Para adicionar a biblioteca 

```sh 
#include "../KL05Z_Libraries/board_frdm_kl05z.h"
```

Inicialização do I/O 

```sh 
gpio_init(D0,GPIO_OUTPUT);  // Inicializa pino D0 como saída
gpio_init(D1,GPIO_INPUT);   // Inicializa pino D1 como entrada
```

Escrevendo no pino

```sh
gpio_write(D0,HIGH);    // Escreve 1 no pino
gpio_write(D0,LOW);     // Escreve 0 no pino
```

Lendo pino 

```sh
gpio_read(D1);    // Lendo entrada 
```

Invertendo o valor do pino 

```sh
gpio_toggle(D0);  // Invertendo valor do pino
```

Inicializando I2C

```sh
i2c_init(I2C_CONFIG);
```

Envindo e recebendo dados pela I2C

```sh
i2c_send_data(I2C,address, reg, data);    // Enviando dados
data = i2c_read_data(I2C,address, reg);   // Recebendo dados
```

Inicializando SPI 

```sh
spi_init(SPI_CONFIG);     // Inicializa Periferico SPI
spi_init_CS(D5);          // Inicializa pino de CS
```

Enviando e recebendo dados pela SPI

```sh
/* Escrevendo dados na SPI */
spi_write_CS(D5,SPI_CS_Enable);
spi_write(SPI, data);
spi_write_CS(D5,SPI_CS_Enable);

/* Lendo dados na SPI */
spi_write_CS(D5,SPI_CS_Enable);
data = spi_read(SPI);
spi_write_CS(D5,SPI_CS_Enable);
```

Inicializando UART (Serial)

```sh
uart_init(UART_CONFIG,9600); 
```

Enviando e recebendo dados pela Serial

```sh
data = uart_getchar(UART);            // Recebe byte de pela serial
uart_getchar(UART,data);              // Enviando byte de pela serial
uart_put_string(UART,"Hello Word");   // Enviando string pela serial
```
