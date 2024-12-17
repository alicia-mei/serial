#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>
#include <zephyr/device.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/drivers/gpio.h>

#include <string.h>
#include <stdio.h>

#define STACKSIZE 1024


/*acesso às fifos................................................................................................................................................. */


/*escritor de fifo*/
void producer(char dado, struct k_fifo my_fifo)
{
	/* Cria novo item pra enviar */
	char escrita = k_malloc(sizeof(escrita));
	printk("malloc\n");
	escrita = dado;
	/* coloca item na fifo */
	printk("bla\n");
	k_fifo_put(&my_fifo, &escrita);
	printk("coloquei na fifo\n");

	//printk("Produtor 1 colocou %d na FIFO (qty_fifo = %d)\n", tx_data->value, contador_fifo);
}

/* leitor de fifo*/
char consumer(struct k_fifo my_fifo)
{
	char leitura = k_fifo_get(&my_fifo, K_FOREVER); //espera até ter alguma coisa para ler na fifo
	char b = 0;
	if (leitura) {
		b = leitura; //guarda o valor de rx_data
		//printk("Consumidor 1 tirou %d da FIFO\n", rx_data->value);
		k_free(leitura); //libera espaço da memória 
	}
	return b; 
}

/*Recepcao da mensagem do usuario.......................................................................................................................................*/
#define UART_DEVICE_NODE DT_CHOSEN(zephyr_shell_uart)
#define MSG_SIZE 8 //tamanho da mensagem em bytes

K_MSGQ_DEFINE(uart_msgq, MSG_SIZE, 10, 4); //fila para armazenar 10 mensagens de tamanho 8 bits, sendo o ultimo bit /0

static const struct device *const uart_dev = DEVICE_DT_GET(UART_DEVICE_NODE); //ponteiro para o dispositivo uart obtido através do nó do device tree

// utilizados na funcao callback serial
static char rx_buf[MSG_SIZE]; //uma string de tamanho 8 bits
static int rx_buf_pos; //variavel usada como buffer da funcao

uint8_t id_mask = 0b00101000;

struct data_item_t {
    void *fifo_reserved;   /* primeira palavra reservada para uso pela FIFO (do site do zephyr)*/
    uint32_t value;        /* valor armazenado */
};

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

int confere_bytes(char info){
	int n_data = 0;
	/*transforma os últimos 3 bits no número de bytes que serão transmitidos*/
	if((info & 0b00000100) == 0b00000100) {
		n_data = 4;
	}
	if((info & 0b00000010) == 0b00000010) {
		n_data = n_data + 2;
	}
	if((info & 0b00000001) == 0b00000001) {
		n_data = n_data + 1;
	}
	printk("n_data = %d\n", n_data);
	return n_data;
}

void get_message_thread()
{
	printk("entrei na thread");
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
	while(1){
    /* espera receber mensagem do usuário para sempre */
		while (k_msgq_get(&uart_msgq, &tx_buf, K_FOREVER) == 0) {
			uint8_t bytes = strlen(tx_buf);  // Número de mensagens a ser convertido em bits para colocar no id 
			char header = id_mask | bytes; //define o cabeçalho (id + qtd de bytes)
			producer(header, header_data); //coloca o cabecalho na fifo
			printk("header: %c \n", header);
			for(int k = 0; k < bytes; k++){
				printf("bytes = %d\n", bytes);
				print_uart(tx_buf);
				struct data_item_t *coisa = k_malloc(sizeof(struct data_item_t));
				if (coisa == NULL) {
					printk("Erro: Falha ao alocar memória!\n");
					return;
				}
				char m = tx_buf[k];
				coisa->value = m; /* o que coloca na fifo */
				printk("value = %c", coisa->value);
				k_fifo_put(&header_data, coisa);/* coloca item na fifo */
				printk("aaaaaaa\n");
				//producer(m, header_data); //coloca os caracteres um a um na fifo
			}
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

int a = 0x0, cnt = 0, *rx_msg = 0, *tx_msg = 0, shift = 0, qtd_bytes = 0, aux = 0, state = 1; 
char sync = 0b00010110, etx = 0b00000011, stx = 0b00000010, cabecalho = 0b0, caracter = 0b0, cancela = 0b00011000, pre = 0b01010101;
uint8_t cabecalho_buf = 0b00101000; 
static char data_buf[MSG_SIZE];
bool achou_sync = 0, nosso = 1;

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

void config_rx_pin_thread(){ //thread pra configurar o pino do rx como input
	int ret;
	printk("pinos \n");
    if (!gpio_is_ready_dt(&button)) {
        printk("Error: button device %s is not ready\n",
               button.port->name);
        //return 0;
    }

    ret = gpio_pin_configure_dt(&button, GPIO_INPUT);
    if (ret != 0) {
        printk("Error %d: failed to configure %s pin %d\n",
               ret, button.port->name, button.pin);
        //return 0;
    }
}

void rx(){
    //printk("rx\n");
	rx_msg = gpio_pin_get_dt(&button); //le o valor recebido no pino que esta o rx
	char c = 0;

	a = a >> 1; //shifta a 1 para a direita
	if(rx_msg == 1) a = a & 0b10000000; //armazena o caracter recebido na variavel a, se for 1, coloca 1, se for 0, deixa 0
	shift++; //incrementa 1 no valor do shift

	if(achou_sync == 0){ //se conseguir pegar o sync_mutex, significa que a thread verifica está procurando o sync
		c = int_to_char(a); //transforma a em um char
		if(c != cancela) producer(c, recebidos); //coloca esse char na fifo de recebidos se ele for válido
	}
	else{ //se não conseguir pegar o sync_mutex, significa que a thread verifica espera receber o resto da mensagem
		if(shift == 32){ //só coloca a palavra na fifo de recebidos depois que ela se formar por completo
			c = int_to_char(a); 
			producer(c, recebidos);
		}
	}
}

void verifica_thread(){
	printk("pass the vibe check\n");
	while(1){
		char caracter = consumer(recebidos);
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
				achou_sync = 1; //para a thread rx saber que o sync foi encontrado
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
				achou_sync = 0;
				state = 0; //se estiver errado, espera acabar a transmissão
			}
		}
		else if(state == 3){ //estado 3: cabecalho
			if(caracter == cancela){ //se for inválido
				achou_sync = 0;
				char lixo = consumer(impressora);
				state = 0; //se estiver errado, volta a procurar 0
			}
			else {
				k_mutex_unlock(&pode_imprimir_mutex); //significa que a transmissão acabou, nosso pode tentar transmitir
				if(caracter == cabecalho_buf){ //compara com o cabecalho armazendo no buf da thread de transmissao
					printk("nosso cabecalho");
					nosso = 1;
					producer(caracter, impressora);
				}
				else{ //se não for nosso id
					nosso = 0;
					producer(caracter, impressora);	
				}
				qtd_bytes = confere_bytes(caracter);
				state = 4; //muda o estado para checar os dados
				aux = 0; //variável usada no próximo estado
			}
		}
		else if(state == 4){ //estado 4: leitura de dados 
			if(aux < qtd_bytes){ //garante que todos os bytes serão lidos
				if(nosso){ //se for nossa transmissão
					if(caracter == data_buf[aux]){ //compara com o cada caracter do dado armazenado no buf da thread de transmissao
						producer(caracter, impressora); //coloca na fifo para imprimir 
					}
					else{
						for(int i = 0; i<aux; i++){
							char lixo = consumer(impressora); //apaga as mensagens que tinha escrito
						}
						achou_sync = 0;
						state = 0; //se não for a mensagem esperada, volta a procurar 0
					}
				} //se não for nossa transmissão
				else{
					if(caracter != cancela){
						producer(caracter, impressora); //armazena a mensagem na fifo
					}
					else{
						achou_sync = 0;
						for(int i = 0; i<aux; i++){
							char lixo = consumer(impressora); //apaga as mensagens que tinha escrito
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
					k_mutex_lock(&next_mutex, K_FOREVER);
					k_condvar_signal(&change_msg); //sinal para permmitir que a proxima mensagem seja enviada, visto que a última estava inteiramente correta
					k_mutex_unlock(&next_mutex); 
				} 
				achou_sync = 0;
				k_mutex_unlock(&tx_mutex); //significa que a transmissão acabou, nosso pode tentar transmitir
				producer(0b00001010, impressora); //coloca um \n para sinalizar que acabou o pacote
				state = 1; // o rx pode voltar a tentar encontrar o sync
			}
			else{
				printk("Falha na recepção da transmissão");
				achou_sync = 0;
				state = 0;
			}
		}
	}
}

void imprimir_thread(){ //thread para imprimir dados
printk("lacrador\n");
	while(1){
		if(k_fifo_peek_tail(&impressora) == 0b00001010){ //se o último item da fifo impressora for um \n
			char p = consumer(impressora); // pega o primeiro valor da fifo
			while(p != 0b00001010){ //enquanto for diferente de \n
				printk(p); //imprime na tela
			}
			printf("\n"); //quebra linha, fim do pacote
		}
	}
}

/*Transmissao................................................................................................................................................. */

int t = 10, pacote[98], j = 0;
const struct device *gpio_rx = DEVICE_DT_GET(DT_NODELABEL(gpiob));

K_MUTEX_DEFINE(pacote_mutex); //cria mutex para saber se pode acessar o pacote ou se ele ainda está sendo feito
K_MUTEX_DEFINE(pode_enviar_mutex); 

K_FIFO_DEFINE(to_be); //cria fifo pra armazenar a próxima mensagem a ser transmitida


int vector_put(char bufft, int pos){ //função para colocar 0 ou 1 no vetor para transmitir 
    //printk("vectooooooooor\n");
	int inicio = pos;
	for(pos; pos < (inicio + 8); pos++){ //coloca a palavra de 8 bits no vetor
        //printk("pos: %d\n", pos);
		if ((bufft & 0b1) == 0b1) pacote[pos] = 1; //se bir for 1, coloca 1
        else pacote[pos] = 0; //se for 0, coloca 0
        bufft = bufft >> 1; //shifta a palavra 1 bit para a direita
    }
	printk("aeh\n");
	return pos; //retorna o valor da última posição do vetor que tenha um número + 1
}

void empacota_thread(){ //thread para criar o vetor com valores para transmitir 
	int i = 1;
	i = vector_put(pre, 1);
	i = vector_put(sync, 9);
	i = vector_put(stx, 17);
	pacote[0] = 2;
	printk("empacotei\n");
    while(1){
		if(k_mutex_lock(&tx_mutex, K_MSEC(1)) == 0){ //se pode transmitir (ninguém está transmitindo)
			k_mutex_unlock(&tx_mutex); //destrava o mutex
			k_mutex_lock(&pode_enviar_mutex, K_FOREVER); //trava o mutex para poder mexer no vetor pacote
        	if(k_fifo_is_empty(&to_be) == 0){ //se houver coisas na fifo para transmitir mensagens
				cabecalho_buf = consumer(to_be); //armazena o valor do cabeçalho a ser transmitido para poder comparar no verifica()
				for(int k = 0; k < confere_bytes(cabecalho_buf); k++){
					data_buf[k] = consumer(to_be);  //armazena o valor dos dados a serem transmitidos para poder comparar no verifica()
				}
				i = vector_put(cabecalho_buf, 25); //coloca o cabeçalho no vetor para transmitir
				for(int k = 0; k < strlen(data_buf); k++){
					i = vector_put(data_buf[k], i); // coloca cada caracter no vetor
				}
				i = vector_put(etx, i); //coloca o etx no vetor
				pacote[i] = 2; //coloca 2 para sinalizar que o pacote acabou
			}
			t = 10; //reseta o tempo de espera para tentar transmitir  
			pacote[0] = 0; //para sinalizar para a função tx que ela pode transmitir alguma coisa
			k_mutex_unlock(&pode_enviar_mutex); //destrava o mutex
		}
		else{
			k_mutex_lock(&pode_enviar_mutex, K_FOREVER); //trava o mutex para poder mexer no pacote
			pacote[0] = 2; //sinaliza que o pacote não pode ser enviado
			k_mutex_unlock(&pode_enviar_mutex); //destrava o mutex
			k_msleep(t); //espera o tempo t
			t = t*2; //duplica o tempo t da próxima espera caso não consiga transmitir
        }
    }
}

void next_message_thread(){ //thread para trocar a mensagem a ser enviada
    printk("thank you next\n");
	while(1){
        k_mutex_lock(&next_mutex, K_FOREVER); //mutex utilizado pela variável condicional
        char h = consumer(header_data); //pega cabecalho da fifo 
        producer(h, to_be); //coloca o header na fifo p enviar
		for(int k = 0; k < confere_bytes(h); k++){
			producer(consumer(header_data), to_be); //coloca o dado na fifo p enviar
		}
        k_condvar_wait(&change_msg, &next_mutex, K_FOREVER); //espera sinal do verifica para pode trocar a mensagem a ser enviada
        k_mutex_unlock(&next_mutex);  //destrava o mutex
    }
}

void tx(){ //função chamada a cada 1/400s para transmitir 
	//printk("tx\n");
	gpio_pin_configure(gpio_rx, 0x3, GPIO_OUTPUT_ACTIVE); //configura gpio para output
	if(pacote[j] != 2){ //se puder transmitir
		if(j>0){ //se estiver no pacote
			if (pacote[j] == 1) gpio_pin_set(gpio_rx ,0x3, 1); //se o bit for 1, transmite 1
			else gpio_pin_set(gpio_rx ,0x3, 0); //se for 0, transmite 0
		}
		j++; //incrementa contador
	}
	else{
		gpio_pin_set(gpio_rx ,0x3, 0); //se não puder transmitir, seta pino como 0
		j = 0; //reinicia o contador
	} 
}


K_TIMER_DEFINE(rx_timer, rx, NULL);
K_TIMER_DEFINE(tx_timer, tx, NULL);

void timers_thread(){
	k_timer_start(&rx_timer, K_USEC(2500), K_USEC(2500)); //timer para chamar a função rx a cada 2500us
	k_timer_start(&tx_timer, K_MSEC(10), K_MSEC(10)); //timer para chamar a função rx a cada 10ms
 	while (1) {
        k_msleep(1000); // Mantém a thread viva
    }
}


/* Definir as threads */
K_THREAD_DEFINE(get_message_id, STACKSIZE, get_message_thread, NULL, NULL, NULL, 5, 0, 0);
K_THREAD_DEFINE(config_rx_pin_id, STACKSIZE, config_rx_pin_thread, NULL, NULL, NULL, 7, 0, 0);
K_THREAD_DEFINE(verifica_id, STACKSIZE, verifica_thread, NULL, NULL, NULL, 5, 0, 0);
K_THREAD_DEFINE(imprimir_id, STACKSIZE, imprimir_thread, NULL, NULL, NULL, 8, 0, 0);
K_THREAD_DEFINE(empacota_id, STACKSIZE, empacota_thread, NULL, NULL, NULL, 7, 0, 0);
K_THREAD_DEFINE(next_message_thread_id, STACKSIZE, next_message_thread, NULL, NULL, NULL, 5, 0, 0);
K_THREAD_DEFINE(timers_thread_id, STACKSIZE, timers_thread, NULL, NULL, NULL, 5, 0, 0);


