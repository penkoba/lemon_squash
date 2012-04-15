//
// remocon (compatible with Buffalo RemoteStation)
// + ranging sensor
//
#define PIN_IR_IN		 8
#define PIN_IR_OUT		12
#define PIN_GENERIC_LED		13
#define PIN_ACTIVE_LED		 7
#define ANALOG_PIN_RANGE_IN	 0

/* REMOCON: output */
#define REMOCON_CMD_DATA_COMPLETION	'E'
#define REMOCON_CMD_LED_OK		'O'
#define REMOCON_CMD_RECEIVE_DATA	'S'
#define REMOCON_CMD_OK			'Y'
/* REMOCON: input */
#define REMOCON_CMD_RECEIVE_CANCEL	'c'
#define REMOCON_CMD_LED			'i'
#define REMOCON_CMD_RECEIVE		'r'
#define REMOCON_CMD_RECEIVE2		's'
#define REMOCON_CMD_TRANSMIT		't'
#define REMOCON_CMD_TRANSMIT2		'u'
#define REMOCON_CMD_CHANNEL(ch)		('0' + (ch))

/* SENSOR: output */
#define SQUASH_CMD_SENSOR_DETECTED	'P'
/* SENSOR: input */
#define SQUASH_CMD_SENSOR_START		'q'
#define SQUASH_CMD_ACTIVE		'a'
#define SQUASH_CMD_INACTIVE		'b'

#define PCOPRS1_DATA_LEN	240

/*--------------------------------------------------------------------*/
static void led_blink(int n, int len)
{
	for (int i = 0; i < n; i++) {
		digitalWrite(PIN_GENERIC_LED, HIGH);
		delay(len);
		digitalWrite(PIN_GENERIC_LED, LOW);
		delay(len);
	}
}

/*--------------------------------------------------------------------*/
class ranging_sensor {
private:
	bool m_enabled;
	uint32_t m_prev_rise_micros;
	uint8_t m_lv;
public:
	ranging_sensor() {
		m_enabled = false;
	}

	void start() {
		m_enabled = true;
		m_lv = 0;
	}
	void stop() {
		m_enabled = false;
	}
	void sense();
};

#define SENSOR_VAL_THRES	300
#define SENSOR_DUR_THRES_US	700000

void ranging_sensor::sense()
{
	char *c;
	uint16_t val;
	uint8_t *val_byte;

	if (!m_enabled)
		return;

	val = analogRead(ANALOG_PIN_RANGE_IN);
	if (val > SENSOR_VAL_THRES) {
		uint32_t now_micros = micros();
		if (m_lv == 0)
			m_prev_rise_micros = now_micros;
		else if (now_micros >= m_prev_rise_micros + SENSOR_DUR_THRES_US) {
			Serial.print(SQUASH_CMD_SENSOR_DETECTED);
			stop();
		}
		m_lv = HIGH;
	} else {
		m_lv = LOW;
	}
}

static ranging_sensor g_rang;

/*--------------------------------------------------------------------*/
class active_indicator {
public:
	void activate() { digitalWrite(PIN_ACTIVE_LED, HIGH); }
	void deactivate() { digitalWrite(PIN_ACTIVE_LED, LOW); }
};

static active_indicator g_aind;

/*--------------------------------------------------------------------*/
class ir_transmitter {
private:
	uint8_t m_data[PCOPRS1_DATA_LEN];
	int m_len;

public:
	void reset() { m_len = 0; }
	int push(uint8_t c);
	void transmit(int len);
};

int ir_transmitter::push(uint8_t c)
{
	m_data[m_len++] = c;
	return m_len;
}

// define 38kHz 1/3 duty signal
#define HI_DUR_US	9
#define LO_DUR_US	17

void ir_transmitter::transmit(int len)
{
	unsigned long cur_us = 0, end_us = 0;

	for (int i = 0; i < len; i++) {
		for (int j = 0; j < 8; j++) {
			// if the bit is set, transmit 38kHz wave for 100us.
			// otherwise keep silent.
			//
			// here we use PORT operation instead of digitalWrite(),
			// for the timing accuracy.
			// digitalWrite() seems to take some microseconds.

			int val = (m_data[i] >> j) & 1;	// we get 0 or 1

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
	void receive(int len);
};

void ir_receiver::receive(int len)
{
	// NOTE: active low
	// FIXME: support cancel command

	// wait for the first falling edge
	while (digitalRead(PIN_IR_IN) == 0);
	while (digitalRead(PIN_IR_IN) != 0);

	// start time
	uint32_t us_prev = micros();

	for (int i = 0; i < len; i++) {
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
		ST_TRANSMIT_SIZE,
		ST_TRANSMIT_CH,
		ST_TRANSMIT,
		ST_RECEIVE_SIZE,
	} m_state;
	ir_transmitter m_tr;
	int m_transmit_size;

public:
	state_machine() {
		m_state = ST_NONE;
	}
	void serial_check();
};

void state_machine::serial_check()
{
	if (Serial.available() == 0)
		return;

	uint8_t c = Serial.read();

	switch (m_state) {
	case ST_NONE:
		switch (c) {
		/* remocon commands */
		case REMOCON_CMD_LED:
			led_blink(1, 100);
			Serial.print(REMOCON_CMD_LED_OK);
			return;
		case REMOCON_CMD_TRANSMIT:
			Serial.print(REMOCON_CMD_OK);
			m_transmit_size = PCOPRS1_DATA_LEN;
			m_state = ST_TRANSMIT_CH;
			return;
		case REMOCON_CMD_TRANSMIT2:
			Serial.print(REMOCON_CMD_OK);
			m_state = ST_TRANSMIT_SIZE;
			return;
		case REMOCON_CMD_RECEIVE:
			{
				ir_receiver rc;
				Serial.print(REMOCON_CMD_OK);
				Serial.print(REMOCON_CMD_RECEIVE_DATA);
				rc.receive(PCOPRS1_DATA_LEN);
				Serial.print(REMOCON_CMD_DATA_COMPLETION);
			}
			return;
		case REMOCON_CMD_RECEIVE2:
			Serial.print(REMOCON_CMD_OK);
			m_state = ST_RECEIVE_SIZE;
			return;
		/* sensor commands */
		case SQUASH_CMD_SENSOR_START:
			g_rang.start();
			return;
		case SQUASH_CMD_ACTIVE:
			g_aind.activate();
			return;
		case SQUASH_CMD_INACTIVE:
			g_aind.deactivate();
			return;
		}
		break;
	case ST_TRANSMIT_SIZE:
		Serial.print(REMOCON_CMD_OK);
		m_transmit_size = (int)c * 16;
		m_state = ST_TRANSMIT_CH;
		return;
	case ST_TRANSMIT_CH:
		if ((c >= '1') && (c <= '4')) {
			Serial.print(REMOCON_CMD_OK);
			m_tr.reset();
			m_state = ST_TRANSMIT;
			return;
		}
		break;
	case ST_TRANSMIT:
		if (m_tr.push(c) == PCOPRS1_DATA_LEN) {
			m_tr.transmit(m_transmit_size);
			Serial.print(REMOCON_CMD_DATA_COMPLETION);
			m_state = ST_NONE;
		}
		return;
	case ST_RECEIVE_SIZE:
		{
			ir_receiver rc;
			Serial.print(REMOCON_CMD_OK);
			Serial.print(REMOCON_CMD_RECEIVE_DATA);
			rc.receive((int)c * 16);
			Serial.print(REMOCON_CMD_DATA_COMPLETION);
		}
		return;
	}

	/* error. unknown command */
	Serial.print('?');
	m_state = ST_NONE;
}

/*--------------------------------------------------------------------*/
void setup()
{
	Serial.begin(115200);
	pinMode(PIN_IR_IN, INPUT);
	pinMode(PIN_IR_OUT, OUTPUT);
	pinMode(PIN_GENERIC_LED, OUTPUT);
	pinMode(PIN_ACTIVE_LED, OUTPUT);
}

void loop()
{
	static state_machine st;

	st.serial_check();
	g_rang.sense();
}
