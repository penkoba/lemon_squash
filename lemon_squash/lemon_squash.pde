//
// remocon (compatible with Buffalo RemoteStation)
//
#define PIN_IR_IN	8
#define PIN_IR_OUT	12
#define PIN_LED		13

#define REMOCON_CMD_DATA_COMPLETION	'E'
#define REMOCON_CMD_LED_OK		'O'
#define REMOCON_CMD_RECEIVE_DATA	'S'
#define REMOCON_CMD_OK			'Y'
#define REMOCON_CMD_RECEIVE_CANCEL	'c'
#define REMOCON_CMD_LED			'i'
#define REMOCON_CMD_RECEIVE		'r'
#define REMOCON_CMD_TRANSMIT		't'
#define REMOCON_CMD_CHANNEL(ch)		('0' + (ch))

#define REMOCON_DATA_LEN	240

/*--------------------------------------------------------------------*/
static void led_blink(int n, int len)
{
	for (int i = 0; i < n; i++) {
		digitalWrite(PIN_LED, HIGH);
		delay(len);
		digitalWrite(PIN_LED, LOW);
		delay(len);
	}
}

/*--------------------------------------------------------------------*/
class ir_transmitter {
private:
	uint8_t data[REMOCON_DATA_LEN];
	int len;

public:
	void reset() { len = 0; }
	int push(uint8_t c);
	void transmit();
};

int ir_transmitter::push(uint8_t c)
{
	data[len++] = c;
	return len;
}

// define 38kHz 1/3 duty signal
#define HI_DUR_US	9
#define LO_DUR_US	17

void ir_transmitter::transmit()
{
	unsigned long cur_us = 0, end_us = 0;

	for (int i = 0; i < REMOCON_DATA_LEN; i++) {
		for (int j = 0; j < 8; j++) {
			// if the bit is set, transmit 38kHz wave for 100us.
			// otherwise keep silent.
			//
			// here we use PORT operation instead of digitalWrite(),
			// for the timing accuracy.
			// digitalWrite() seems to take some microseconds.

			int val = (data[i] >> j) & 1;	// we get 0 or 1

			// port B: pin 8-13
			// the shift count is (PIN_IR_OUT - 8)
			uint8_t d0 = PORTB;
			uint8_t d1 = d0 | (val << (PIN_IR_OUT - 8));

			end_us += 100;	// repeat below for 100us
			for (;
			     cur_us < end_us;
			     cur_us += HI_DUR_US + LO_DUR_US) {
				PORTB = d1;
				delayMicroseconds(HI_DUR_US);
				PORTB = d0;
				delayMicroseconds(LO_DUR_US);
			}
		}
	}
	led_blink(2, 50);
}

/*--------------------------------------------------------------------*/
class ir_receiver {
public:
	void receive();
};

void ir_receiver::receive()
{
	// NOTE: active low
	// FIXME: support cancel command

	// wait for the first falling edge
	while (digitalRead(PIN_IR_IN) == 0);
	while (digitalRead(PIN_IR_IN) != 0);

	// start time
	uint32_t us_prev = micros();

	for (int i = 0; i < REMOCON_DATA_LEN; i++) {
		uint8_t rcv_byte = 0;
		for (int j = 0; j < 8; j++) {
			uint32_t us_next = us_prev + 100;
			rcv_byte |= (digitalRead(PIN_IR_IN) << j);
			// wait for 100us
			while (micros() < us_next);
			us_prev = us_next;
		}
		Serial.write(~rcv_byte);
	}

	led_blink(2, 50);
}

/*--------------------------------------------------------------------*/
class state_machine {
private:
	enum {
		ST_NONE = 0,
		ST_TRANSMIT_CH,
		ST_TRANSMIT,
		ST_RECEIVE,
	};

	ir_transmitter tr;
	int state;

public:
	state_machine() {
		state = ST_NONE;
	}
	void serial_check();
};

void state_machine::serial_check()
{
	if (Serial.available() == 0)
		return;

	uint8_t c = Serial.read();

	if (state == ST_NONE) {
		if (c == REMOCON_CMD_LED) {
			led_blink(1, 100);
			Serial.print(REMOCON_CMD_LED_OK);
		} else if (c == REMOCON_CMD_TRANSMIT) {
			Serial.print(REMOCON_CMD_OK);
			state = ST_TRANSMIT_CH;
		} else if (c == REMOCON_CMD_RECEIVE) {
			ir_receiver rc;
			Serial.print(REMOCON_CMD_OK);
			Serial.print(REMOCON_CMD_RECEIVE_DATA);
			rc.receive();
			Serial.print(REMOCON_CMD_DATA_COMPLETION);
		} else
			goto err;
	} else if (state == ST_TRANSMIT_CH) {
		if ((c >= '1') && (c <= '4')) {
			Serial.print(REMOCON_CMD_OK);
			tr.reset();
			state = ST_TRANSMIT;
		} else
			state = ST_NONE;
	} else if (state == ST_TRANSMIT) {
		if (tr.push(c) == REMOCON_DATA_LEN) {
			tr.transmit();
			Serial.print(REMOCON_CMD_DATA_COMPLETION);
			state = ST_NONE;
		}
	}

	return;

err:
	state = ST_NONE;
	Serial.print('?');
}

/*--------------------------------------------------------------------*/
void setup()
{
	Serial.begin(115200);
	pinMode(PIN_IR_IN, INPUT);
	pinMode(PIN_IR_OUT, OUTPUT);
	pinMode(PIN_LED, OUTPUT);
}

void loop()
{
	static state_machine st;

	st.serial_check();
}
