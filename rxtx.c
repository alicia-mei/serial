#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/drivers/gpio.h>

#include <string.h>
#include <stdio.h>

#define STACKSIZE 1024

K_FIFO_DEFINE(my_fifo);
K_MUTEX_DEFINE(my_mutex);

/*acesso às fifos................................................................................................................................................. */

struct data_item_t {
    void *fifo_reserved;   /* primeira palavra reservada para uso pela FIFO (do site do zephyr)*/
    char value;        /* valor armazenado */
};

/*escritor 1*/
void producer_thread(char dado, struct k_fifo *fifo)
{
	/* Cria novo item pra enviar */
	struct data_item_t *tx_data = k_malloc(sizeof(struct data_item_t));

	tx_data->value = dado; /* o que o escritor 1 coloca na fifo */

	/* coloca item na fifo */
	k_fifo_put(&fifo, &tx_data);

	//printk("Produtor 1 colocou %d na FIFO (qty_fifo = %d)\n", tx_data->value, contador_fifo);
}

/* leitor 1*/
char consumer_thread(struct k_fifo *fifo)
{
	struct data_item_t *rx_data = k_fifo_get(&fifo, K_FOREVER); //espera até ter alguma coisa para ler na fifo

	if (rx_data) {
	char b = rx_data->value; //guarda o valor de rx_data
	//printk("Consumidor 1 tirou %d da FIFO\n", rx_data->value);
		k_free(rx_data); //libera espaço da memória 
	}
	return b; 
}


/*Recepcao da transmissao................................................................................................................................................. */
#define SW0_NODE    DT_ALIAS(sw0) //vamos utilizar o mesmo pino do botao como gpio de input
#if !DT_NODE_HAS_STATUS_OKAY(SW0_NODE)
#error "Unsupported board: sw0 devicetree alias is not defined"
#endif

static const struct gpio_dt_spec button = GPIO_DT_SPEC_GET_OR(SW0_NODE, gpios,
                                  {0});
static struct gpio_callback button_cb_data;

int a = 0x0, cnt = 0, *rx_msg = 0, *tx_msg = 0, shift = 0, n_data = 0, aux = 0, state = 1; 
char sync = 0b00010110, etx = 0b00000011, stx = 0b00000010, cabecalho = 0b0, caracter = 0b0, cancela = 0b00011000;
uint8_t cabecalho_buf = 0b00101000; 
bool achou_sync = 0;

K_MUTEX_DEFINE(tx_mutex); //cria mutex para travar a nossa transmissão
K_MUTEX_DEFINE(sync_mutex); //cria mutex para ver se ta procurando o sync ou não
K_MUTEX_DEFINE(msg_mutex); //cria mutex para travar a leitura e escrita da mensagem no data_buf e cabecalho_buf
K_MUTEX_DEFINE(next_mutex); //cria mutex para travar a mudanca de mensagem
K_MUTEX_DEFINE(pode_imprimir_mutex); //cria mutex para travar a impressão da mensagem
K_FIFO_DEFINE(recebidos); //cria fifo pra armazenar o cabecalho + dados
K_FIFO_DEFINE(impressora); //cria fifo pra armazenar os valores que podem ser impressos
K_CONDVAR_DEFINE(change_msg);

char int_to_char(int mensagem){
	char transformado = 0;
	for(int i = 0; i < 8; i++){
		if((mensagem & 0b0110) == 0b0110){ //compara com os últimos 4 bits do int, se for 1
			transformado >> 1; //shifta 1 pro lado
			transformado = transformado | 0b100000000; //coloca 1 no bit mais significativo do char
		}
		else if((mensagem & 0b0110) == 0b0110) transformado >> 1; //se for 0, shitfta 1 pro lado e não coloca nada
		else return cancela;
		mensagem >> 4; // shifta 4 bits da mensagem
	}
	return transformado;
}

void config_rx_pin(){ //thread pra configurar o pino do rx como input
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
}

void rx(){
    
	rx_msg = gpio_pin_get_dt(&button); //le o valor recebido no pino que esta o rx
	char c = 0;

	a = a << 1; //shifta a 1 para a esquerda
	a = a | rx_msg; //armazena o caracter recebido na variavel a
	shift++; //incremente 1 no valor do sync

	if(k_mutex_lock(&sync_mutex, K_MSEC(1)) == 0){ //se conseguir pegar o sync_mutex, significa que a thread verifica está procurando o sync
		c = int_to_char(a); //transforma a em um char
		if(c != cancela) producer_thread(c, &recebidos); //coloca esse char na fifo de recebidos se ele for válido
		k_mutex_unlock(&sync_mutex); //solta o mutex
	}
	else{ //se não conseguir pegar o sync_mutex, significa que a thread verifica espera receber o resto da mensagem
		if(shift == 32){ //só coloca a palavra na fifo de recebidos depois que ela se formar por completo
			c = int_to_char(a); 
			producer_thread(c, &recebidos);
		}
	}
}

void verifica(){
	while(1){
		char caracter = consumer_thread(&recebidos);
		 /*máquina de estados para checar qual informação está recebendo*/
		if(state == 0){
			if(caracter == 0){ //significa que ninguém está transmitindo nada
				k_mutex_unlock(&tx_mutex); //nosso pode tentar transmitir
				state = 1;
			}
		}
		else if(state == 1){
			if(caracter == sync){ //é o sync
				printk("sync");
				k_mutex_lock(&sync_mutex, K_FOREVER); //trava o mutex para a thread rx saber que o sync foi encontrado
				k_mutex_lock(&tx_mutex, K_FOREVER); //achou uma possível transmissão, impede tentar começar uma nova transmissão nossa
				state = 2;
			}
		}   
		else if(state == 2){ //estado 2: stx
			if(caracter == stx){
				printk("stx");
				state = 3; //se estiver correto, muda estado para conferir o cabecalho
			}
			else {
				k_mutex_unlock(&sync_mutex); //solta o mutex
				state = 0; //se estiver errado, espera acabar a transmissão
			}
		}
		else if(state == 3){ //estado 3: cabecalho
			if(caracter == cancela){ //se for inválido
				k_mutex_unlock(&sync_mutex); //solta o mutex
				char lixo = consumer_thread(&impressora);
				state = 0; //se estiver errado, volta a procurar 0
			}
			else {
				k_mutex_unlock(&pode_imprimir_mutex); //significa que a transmissão acabou, nosso pode tentar transmitir
				if(caracter == cabecalho_buf){ //compara com o cabecalho armazendo no buf da thread de transmissao
					printk("nosso cabecalho");
					nosso = 1;
					producer_thread(caracter, &impressora);
				}
				else{ //se não for nosso id
					nosso = 0;
					producer_thread(caracter, &impressora);	
				}
				n_data = 0;
				/*transforma os últimos 3 bits no número de bytes que serão transmitidos*/
				if((caracter & 0b00000100) == 0b00000100) {
					n_data = 4;
				}
				if((caracter & 0b00000010) == 0b00000010) {
					n_data = n_data + 2;
				}
				if((caracter & 0b00000001) == 0b00000001) {
					n_data = n_data + 1;
				}
				state = 4; //muda o estado para checar os dados
				aux = 0; //variável usada no próximo estado
			}
		}
		else if(state == 4){ //estado 4: leitura de dados 
			if(aux < n_data){ //garante que todos os bytes serão lidos
				if(nosso){ //se for nossa transmissão
					if(caracter == data_buf[aux]){ //compara com o cada caracter do dado armazenado no buf da thread de transmissao
						producer_thread(caracter, &impressora); //coloca na fifo para imprimir 
					}
					else{
						for(int i = 0; i<aux; i++){
							char lixo = consumer_thread(&impressora); //apaga as mensagens que tinha escrito
						}
						k_mutex_unlock(&sync_mutex); //solta o mutex
						state = 0; //se não for a mensagem esperada, volta a procurar 0
					}
				} //se não for nossa transmissão
				else{
					if(caracter != cancela){
						producer_thread(caracter, &impressora); //armazena a mensagem na fifo
					}
					else{
						k_mutex_unlock(&sync_mutex); //solta o mutex
						for(int i = 0; i<aux; i++){
							char lixo = consumer_thread(&impressora); //apaga as mensagens que tinha escrito
						}
						state = 0;
					}
				}
				aux++; //incremente o contador auxiliar
			}
			else state = 5; //se recebeu todas as mensagems que deveria receber, muda o estado
		}
		else if(state == 5){//estado 5: etx
			if(caracter == etx){
				printk("etx");
				if (nosso == 1){
					k_mutex_lock(&next_mutex);
					k_condvar_signal(&change_msg); //sinal para permmitir que a proxima mensagem seja enviada, visto que a última estava inteiramente correta
					k_mutex_unlock(&next_mutex); 
				} 
				k_mutex_unlock(&sync_mutex); //solta o mutex
				k_mutex_unlock(&tx_mutex); //significa que a transmissão acabou, nosso pode tentar transmitir
				producer_thread(0b00001010, &impressora); //coloca um \n para sinalizar que acabou o pacote
				state = 1; // o rx pode voltar a tentar encontrar o sync
			}
			else{
				printk("Falha na recepção da transmissão");
				k_mutex_unlock(&sync_mutex); //solta o mutex
				state = 0;
			}
		}
	}
}

void imprimir(){ //thread para imprimir dados
	while(1){
		if(k_fifo_peak_tail(&impressora) == 0b00001010){ //se o último item da fifo impressora for um \n
			p = consumer_thread(&impressora); // pega o primeiro valor da fifo
			while(p != 0b00001010){ //enquanto for diferente de \n
				printk(p); //imprime na tela
			}
			print("\n"); //quebra linha, fim do pacote
		}
	}
}

/*Transmissao................................................................................................................................................. */

const struct device *gpio_rx = DEVICE_DT_GET(DT_NODELABEL(gpiob));

K_FIFO_DEFINE(to_be); //cria fifo pra armazenar a próxima mensagem a ser transmitida

void write_int(int bufft){
    for(int i = 0; i < 8; i++){
        if ((bufft & 0b10000000) == 0b10000000) gpio_pin_set(gpio_rx ,0x3, 1);
        else gpio_pin_set(gpio_rx ,0x3, 0);
        wait_us(100);
        bufft = bufft << 1;
    }
}

void write_char(char bufft){
    for(int i = 0; i < 8; i++){
        if ((bufft & 0b10000000) == 0b10000000) gpio_pin_set(gpio_rx ,0x3, 1);
        else gpio_pin_set(gpio_rx ,0x3, 0);
        wait_us(100);
        bufft = bufft << 1;
    }
}

void tx(){
    gpio_pin_configure(gpio_rx, 0x3, GPIO_OUTPUT_ACTIVE);
    while(1){
        if(k_fifo_is_empty(&to_be) == 0){ //se houver coisas na fifo para transmitir mensagens
            cabecalho_buf = k_fifo_get(&to_be, K_FOREVER);
            data_buf = k_fifo_get(&to_be, K_FOREVER);
            write_
            write_int(cabecalho_buf); 
            write_char(data_buf);  
        }
    }
}

void next_message(){
    while(1){
        k_mutex_lock(&next_mutex, K_FOREVER);
        int h = k_fifo_get(&header_data, K_FOREVER); //pega cabecalho da fifo 
        k_fifo_put(&to_be, &h); //coloca o header na fifo p enviar
        char d = k_fifo_get(&header_data, K_FOREVER); //pega dado da fifo 
        k_fifo_put(&to_be, &d); //coloca o dado na fifo p enviar
        k_condvar_wait(&change_msg, &next_message, K_FOREVER);
        k_mutex_unlock(&next_mutex);
    }
}

/* incompleto */

K_TIMER_DEFINE(rx_timer, rx, NULL);
k_timer_start(&rx_timer, K_MSEC(200), K_MSEC(200)); //timer para chamar a função rx a cada 200ms

/* Definir as threads */
/* Obs.: para obter as diferentes combinações, basta comentar o define das threads que não deseja utilizar*/
K_THREAD_DEFINE(get_message_id, STACKSIZE, get_message_thread, NULL, NULL, NULL, 7, 0, 0);
K_THREAD_DEFINE(producer_1_id, STACKSIZE, producer_thread_1, NULL, NULL, NULL, 7, 0, 0);
K_THREAD_DEFINE(consumer_1_id, STACKSIZE, consumer_thread_1, NULL, NULL, NULL, 7, 0, 0);
