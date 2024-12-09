#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/drivers/gpio.h>

#include <string.h>
#include <stdio.h>

#define STACKSIZE 1024

K_FIFO_DEFINE(my_fifo);
K_MUTEX_DEFINE(my_mutex);

struct data_item_t {
    void *fifo_reserved;   /* primeira palavra reservada para uso pela FIFO (do site do zephyr)*/
    uint32_t value;        /* valor armazenado */
};

/* Contador de itens da fifo, max 11 */
int qty_fifo_header_data = 0;

/*Recepcao da mensagem do usuario.......................................................................................................................................*/
#define UART_DEVICE_NODE DT_CHOSEN(zephyr_shell_uart)
#define MSG_SIZE 8 //tamanho da mensagem em bytes

K_MSGQ_DEFINE(uart_msgq, MSG_SIZE, 10, 4); //fila para armazenar 10 mensagens de tamanho 8 bits, sendo o ultimo bit /0

static const struct device *const uart_dev = DEVICE_DT_GET(UART_DEVICE_NODE); //ponteiro para o dispositivo uart obtido através do nó do device tree

// utilizados na funcao callback serial
static char rx_buf[MSG_SIZE]; //uma string de tamanho 8 bits
static int rx_buf_pos; //variavel usada como buffer da funcao

uint8_t id_mask = 0b00101000;

K_FIFO_DEFINE(header_data); //cria fifo pra armazenar o cabecalho + dados

void serial_cb(const struct device *dev, void *user_data)
{
    uint8_t c;

    if (!uart_irq_update(uart_dev)) {
        return;
    }

    if (!uart_irq_rx_ready(uart_dev)) {
        return;
    }

    /* le ate a fifo ficar vazia */
    while (uart_fifo_read(uart_dev, &c, 1) == 1) {
        if ((c == '\n' || c == '\r') && rx_buf_pos > 0) { //se o caracter lido for \n ou \r
            rx_buf[rx_buf_pos] = '\0'; //termina a string

            k_msgq_put(&uart_msgq, &rx_buf, K_NO_WAIT); //coloca a mensagem na fila de mensagens

            rx_buf_pos = 0; //reseta o buffer para 0
        } else {
            rx_buf[rx_buf_pos++] = c; //addiciona o caracter lido na string de buffer na posição rx_buf_pos e depois soma 1 a rx_buf_pos

            if (rx_buf_pos >= MSG_SIZE - 1) { //se ja foram lidos 7 chars
                rx_buf[rx_buf_pos] = '\0';  //termina a string
                k_msgq_put(&uart_msgq, &rx_buf, K_NO_WAIT); // coloca na fila de mensagens
                rx_buf_pos = 0;  // reseta o buffer para poder continuar recebendo a mensagem
            }
        }
    }
}

void print_uart(char *buf)
{
    int msg_len = strlen(buf); //tamanho da mensagem

    for (int i = 0; i < msg_len; i++) {
        uart_poll_out(uart_dev, buf[i]); //printa a mensagem
    }
}

void get_message()
{
    char tx_buf[MSG_SIZE]; //string de 8 bytes

    if (!device_is_ready(uart_dev)) {
        printk("UART device not found!\n");
        return 0;
    }

    /* configura interrupt e callback para receber dados*/
    int ret = uart_irq_callback_user_data_set(uart_dev, serial_cb, NULL);

    if (ret < 0) { //erros
        if (ret == -ENOTSUP) {
            printk("Interrupt-driven UART API support not enabled\n");
        } else if (ret == -ENOSYS) {
            printk("UART device does not support interrupt-driven API\n");
        } else {
            printk("Error setting UART callback: %d\n", ret);
        }
        return 0;
    }
    uart_irq_rx_enable(uart_dev); //ativa a uart
    
    print_uart("Digite a mensagem que deseja enviar e aperte enter ao terminar\r\n");

    /* espera receber mensagem do usuário para sempre */
    while (k_msgq_get(&uart_msgq, &tx_buf, K_FOREVER) == 0) {
        if(qty_fifo_header_data > 20){
            uint8_t bits = strlen(tx_buf);  // Número de mensagens a ser convertido em bits para colocar no id 
            uint8_t header = id_mask | bits; //define o cabeçalho (id + qtd de bytes)
            k_fifo_put(&header_data, &header); //coloca o cabecalho na fifo
            k_fifo_put(&header_data, &tx_buf); //coloca a mensagem na fifo
        }
        else{
            print_uart("existem muitas mensagens na fila de transmissão, tente novamente mais tarde.\r\n");
        }
    }
}

/*Recepcao da transmissao................................................................................................................................................. */
#define SW0_NODE    DT_ALIAS(sw0) //vamos utilizar o mesmo pino do botao como gpio de input
#if !DT_NODE_HAS_STATUS_OKAY(SW0_NODE)
#error "Unsupported board: sw0 devicetree alias is not defined"
#endif

static const struct gpio_dt_spec button = GPIO_DT_SPEC_GET_OR(SW0_NODE, gpios,
                                  {0});
static struct gpio_callback button_cb_data;

int a = 0x0, cnt = 0, *rx_msg = 0, *tx_msg = 0, shift = 0, n_data, aux = 0; 
char sync = 0b00010110, etx = 0b00000011, stx = 0b00000010, cabecalho = 0b0, caracter = 0b0;
char cabecalho_buf = 0b00101000; 

K_MUTEX_DEFINE(tx_mutex);
K_FIFO_DEFINE(recebidos); //cria fifo pra armazenar o cabecalho + dados

void rx(){
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
    while(1){
        rx_msg = gpio_pin_get_dt(&button); //le o valor recebido no pino que esta o rx

        a = a << 1; //shifta a 1 para a esquerda
        a = a | rx_msg; //armazena o caracter recebido na variavel a

        /*cnt = 3 significa que o dispositivo deve ter recebido o mesmo numero 4 vezes*/
        if(cnt == 3){
            if(((a & 0b1111) == 0b1111) || ((a & 0b1111) == 0b0111) || ((a & 0b1111) == 0b1110) || ((a & 0b1111) == 0b0110)){ //mascaras para verificar se os dois bits centrais são iguais a 1
                caracter = caracter << 1; //shifta uma posicao para a esquerda para poder armazenar um valor no ultimo bit
                caracter = caracter | 0b1; //armazena 1 no ultimo bit
                cnt = 0; //zera o contador
                if(state > 1) shift++; //contador para saber quantos bits foram armazenados em caracter
            }
            else if(((a & 0b1111) == 0b0) || ((a & 0b1111) == 0b0001) || ((a & 0b1111) == 0b1000) || ((a & 0b1111) == 0b1001)){ //mascaras para verificar se os dois bits centrais são iguais a 0
                caracter = caracter << 1;  //shifta uma posicao, 0 fica no ultimo bit
                cnt = 0;
                if(state > 1) shift++;
            }
        }
        else {
            cnt++;
        }
        /*máquina de estados para checar qual informação está recebendo*/
        if(state == 0){ //estado 0: pre
            if(caracter == pre){
                printk("pre");
                state = 1; //se estiver correto, muda estado para conferir o sync
                shift = 0; //inicia contagem de quantos bits foram armazenados para comparar o caracter criado com o esperado
            }
        }
        else if(shift = 8){
            if(state == 1){ //estado 1: sync
                if(caracter == sync){
                    printk("sync");
                    state = 2; //se estiver correto, muda estado para conferir o stx
                }
                else state = 0; //se o dado estiver incorreto, reinicia a maquina de estados, volta a procurar o pre
            }
            else if(state == 2){ //estado 2: stx
                if(caracter == stx){
                    printk("stx");
                    state = 3; //se estiver correto, muda estado para conferir o cabecalho
                }
                else state = 0;
            }
            else if(state == 3){ //estado 3: cabecalho
                if(caracter == cabecalho_buf){ //compara com o cabecalho armazendo no buf da thread de transmissao
                    printk("nosso cabecalho");
                    /*transforma os últimos 3 bits no número de bytes que serão transmitidos*/
                    if((caracter & 0b00000100) == 0b00000100) {
                        n_data = n_data + 4;
                    }
                    if((caracter & 0b00000010) == 0b00000010) {
                        n_data = n_data + 2;
                    }
                    if((caracter & 0b00000001) == 0b00000001) {
                        n_data = n_data + 1;
                    }
                    state = 4; //muda o estado para checar os dados
                }
                else{
                    state = 0; //se não for nosso id
                    if(k_mutex_lock(&tx_mutex, K_FOREVER) == 0); //trava o mutex da transmissao, nosso para de transmitir
                }
            }
            else if(state == 4){ //estado 4: leitura de dados
                if(aux < n_data){ //garante que todos os bytes serão lidos
                    if(caracter == data_buf[aux]){ //compara com o cada caracter do dado armazenado no buf da thread de transmissao
                        k_fifo_put(&recebidos, &caracter); //coloca o caracter na fifo de recebidos
                        aux++; //incremente o contador auxiliar
                    }
                    else state = 0;
                }
                else state 
            }
            else if(state == 5){//estado 5: etx
                if(caracter == ){
                    printk("etx");
                }
                state = 0;
                //signal para a thread de mudança poder atualizar a fifo tx
            }
        }
    }
}


/*Transmissao................................................................................................................................................. */

const struct device *gpio_rx = DEVICE_DT_GET(DT_NODELABEL(gpiob));

void tx(){
    gpio_pin_configure(gpio_rx, 0x3, GPIO_OUTPUT_ACTIVE);
    gpio_pin_set(gpio_rx ,0x3, valor para setar o pino);


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
