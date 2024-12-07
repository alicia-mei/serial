#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/drivers/gpio.h>

#include <string.h>
#include <stdio.h>

#define STACKSIZE 1024

#define UART_DEVICE_NODE DT_CHOSEN(zephyr_shell_uart)

#define MSG_SIZE 8 //tamanho da mensagem em bytes

#define SW0_NODE    DT_ALIAS(sw0)
#if !DT_NODE_HAS_STATUS_OKAY(SW0_NODE)
#error "Unsupported board: sw0 devicetree alias is not defined"
#endif
static const struct gpio_dt_spec button = GPIO_DT_SPEC_GET_OR(SW0_NODE, gpios,
                                  {0});
static struct gpio_callback button_cb_data;

const struct device *stx = DEVICE_DT_GET(DT_NODELABEL(gpiob));

/* no int main colocar
    gpio_pin_configure(stx, 0x3, GPIO_OUTPUT_ACTIVE);
    gpio_pin_set(stx,0x3,valor para setar o pino);

*/

K_FIFO_DEFINE(my_fifo);
K_MUTEX_DEFINE(my_mutex);

/* queue to store up to 10 messages (aligned to 4-byte boundary) */
K_MSGQ_DEFINE(uart_msgq, MSG_SIZE, 10, 4);

static const struct device *const uart_dev = DEVICE_DT_GET(UART_DEVICE_NODE);

/* receive buffer used in UART ISR callback */
static char rx_buf[MSG_SIZE];
static int rx_buf_pos;

struct data_item_t {
    void *fifo_reserved;   /* primeira palavra reservada para uso pela FIFO (do site do zephyr)*/
    uint32_t value;        /* valor armazenado */
};

int a = 0x0, cnt = 0, *rx_msg = 0, *tx_msg = 0; 

/* Contador de itens da fifo, max 11 */
int qty_fifo = 0;

/*
 * Read characters from UART until line end is detected. Afterwards push the
 * data to the message queue.
 */

/*....................................................................................................................................................................*/
void serial_cb(const struct device *dev, void *user_data)
{
	uint8_t c;

	if (!uart_irq_update(uart_dev)) {
		return;
	}

	if (!uart_irq_rx_ready(uart_dev)) {
		return;
	}

	/* read until FIFO empty */
	while (uart_fifo_read(uart_dev, &c, 1) == 1) {
		if ((c == '\n' || c == '\r') && rx_buf_pos > 0) {
			/* terminate string */
			rx_buf[rx_buf_pos] = '\0';

			/* if queue is full, message is silently dropped */
			k_msgq_put(&uart_msgq, &rx_buf, K_NO_WAIT);

			/* reset the buffer (it was copied to the msgq) */
			rx_buf_pos = 0;
		} else if (rx_buf_pos < (sizeof(rx_buf) - 1)) {
			rx_buf[rx_buf_pos++] = c;
		}
		/* else: characters beyond buffer size are dropped */
	}
}

/*
 * Print a null-terminated string character by character to the UART interface
 */
void print_uart(char *buf)
{
	int msg_len = strlen(buf);

	for (int i = 0; i < msg_len; i++) {
		uart_poll_out(uart_dev, buf[i]);
	}
}

void get_message_thread(int unused1, int unused2, int unused3)
{
    char tx_buf[MSG_SIZE];

	if (!device_is_ready(uart_dev)) {
		printk("UART device not found!");
		return 0;
	}

	/* configure interrupt and callback to receive data */
	int ret = uart_irq_callback_user_data_set(uart_dev, serial_cb, NULL);

	if (ret < 0) {
		if (ret == -ENOTSUP) {
			printk("Interrupt-driven UART API support not enabled\n");
		} else if (ret == -ENOSYS) {
			printk("UART device does not support interrupt-driven API\n");
		} else {
			printk("Error setting UART callback: %d\n", ret);
		}
		return 0;
	}
	uart_irq_rx_enable(uart_dev);

	print_uart("Digite a mensagem que deseja enviar e depois clique enter: \r\n");

	/* indefinitely wait for input from the user */
	while (k_msgq_get(&uart_msgq, &tx_buf, K_FOREVER) == 0) {
		print_uart("voce digitou: ");
		print_uart(tx_buf);
		print_uart("\r\n");
        for(int i = 0; i < strlen(tx_buf); i++){

        }
	}
	return 0;
}

/*................................................................................................................................................................... */

void rx(){
    const struct device *dev;
    int ret;

    if (!gpio_is_ready_dt(&button)) {
        printk("Error: button device %s is not ready\n",
               button.port->name);
        return 0;
    }

    ret = gpio_pin_configure_dt(&button, GPIO_INPUT);
    if (ret != 0) {
        printk("Error %d: failed to configure %s pin %d\n",
               ret, button.port->name, button.pin);
        return 0;
    }

    gpio_port_get(&button, &rx_msg);
    a = a | rx_msg; //armazena o caracter recebido na variavel a

    /*cnt = 3 significa que o dispositivo deve ter recebido o mesmo numero 4 vezes*/
    if(cnt == 3){
        if(((a & 0b1111) == 0b1111) || ((a & 0b1111) == 0b0111) || ((a & 0b1111) == 0b1110) || ((a & 0b1111) == 0b0110)){ //mascaras para verificar se os dois bits centrais são iguais a 1
            caracter = caracter << 1; //shifta uma posicao para a esquerda para poder armazenar um valor no ultimo bit
            caracter = caracter | 0b1; //armazena 1 no ultimo bit
            cnt = 0; //zera o contador
        }
        else if(((a & 0b1111) == 0b0) || ((a & 0b1111) == 0b0001) || ((a & 0b1111) == 0b1000) || ((a & 0b1111) == 0b1001)){ //mascaras para verificar se os dois bits centrais são iguais a 0
            caracter = caracter << 1;  //shifta uma posicao, 0 fica no ultimo bit
            cnt = 0;
        }
    }
    else {
        cnt++;
    }
}

void tx(char palavra){
    char bufft = palavra;
    for(int i = 0; i < 8; i++){
        if ((bufft & 0b10000000) == 0b10000000) tx.write(1); 
        else tx.write(0);
        wait_us(100);
        bufft = bufft << 1;
    }
}

/*escritor 1*/
void producer_thread_1(int unused1, int unused2, int unused3)
{
    while (1) {
        if (qty_fifo < 11) { //se ainda há espaço na fifo
            if (k_mutex_lock(&my_mutex, K_MSEC(100)) == 0) { //pega o mutex
                /* Cria novo item pra enviar */
                struct data_item_t *tx_data = k_malloc(sizeof(struct data_item_t));

                tx_data->value = 4; /* o que o escritor 1 coloca na fifo */

                /* coloca item na fifo */
                k_fifo_put(&my_fifo, tx_data);
                qty_fifo++; // aumenta o valor do contador
                printk("Produtor 1 colocou %d na FIFO (qty_fifo = %d)\n", tx_data->value, qty_fifo);

                k_mutex_unlock(&my_mutex); //destrava o mutex
				k_msleep(1000);
            }
        } else {
            k_msleep(100); 
        }
    }
}

/* escritor 2 - mesma lógica do escritor 1*/
void producer_thread_2(int unused1, int unused2, int unused3)
{
    while (1) {
        if (qty_fifo < 11) {
            if (k_mutex_lock(&my_mutex, K_MSEC(100)) == 0) {
                struct data_item_t *tx_data = k_malloc(sizeof(struct data_item_t));

                tx_data->value = 8; 

                k_fifo_put(&my_fifo, tx_data);
                qty_fifo++;
                printk("Produtor 2 colocou %d na FIFO (qty_fifo = %d)\n", tx_data->value, qty_fifo);

                k_mutex_unlock(&my_mutex);
				k_msleep(1000);
            }
        } else {
            k_msleep(100);
        }
    }
}

/* leitor 1*/
void consumer_thread_1(int unused1, int unused2, int unused3)
{
    while (1) {
        struct data_item_t *rx_data = k_fifo_get(&my_fifo, K_FOREVER); //espera até ter alguma coisa para ler na fifo

        if (rx_data) {
            printk("Consumidor 1 tirou %d da FIFO\n", rx_data->value);

			/*trava o mutex para que só uma thread mude o valor de qty_fifo por vez*/
            if (k_mutex_lock(&my_mutex, K_MSEC(100)) == 0) {
                qty_fifo--;
                k_mutex_unlock(&my_mutex); // destrava o mutex
				k_msleep(1000);
            }

           k_free(rx_data);
        }
    }
}

/*leitor 2 - usa a mesma lógica do leitor 2*/
void consumer_thread_2(int unused1, int unused2, int unused3)
{
    printk("Consumidor 2 iniciado\n");
    while (1) {
        struct data_item_t *rx_data = k_fifo_get(&my_fifo, K_FOREVER);

        if (rx_data) {
            printk("Consumidor 2 tirou %d da FIFO\n", rx_data->value);

            if (k_mutex_lock(&my_mutex, K_MSEC(100)) == 0) {
                qty_fifo--;
                k_mutex_unlock(&my_mutex);
				k_msleep(1000);
            }

           k_free(rx_data);
        }
    }
}

/* incompleto */

K_TIMER_DEFINE(rx_timer, rx, NULL);
k_timer_start(&rx_timer, K_MSEC(200), K_MSEC(200)); //timer para chamar a função rx a cada 200ms

/* Definir as threads */
/* Obs.: para obter as diferentes combinações, basta comentar o define das threads que não deseja utilizar*/
K_THREAD_DEFINE(get_message_id, STACKSIZE, get_message_thread, NULL, NULL, NULL, 7, 0, 0);
K_THREAD_DEFINE(producer_1_id, STACKSIZE, producer_thread_1, NULL, NULL, NULL, 7, 0, 0);
K_THREAD_DEFINE(producer_2_id, STACKSIZE, producer_thread_2, NULL, NULL, NULL, 7, 0, 0);
K_THREAD_DEFINE(consumer_1_id, STACKSIZE, consumer_thread_1, NULL, NULL, NULL, 7, 0, 0);
K_THREAD_DEFINE(consumer_2_id, STACKSIZE, consumer_thread_2, NULL, NULL, NULL, 7, 0, 0);
