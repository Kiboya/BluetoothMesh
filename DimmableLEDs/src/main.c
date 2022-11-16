/**
 * Dimmable LEDs - behaviour predominantly based on the Bluetooth mesh Generic OnOff Client Model
 * 
 * Coded for and tested with nRF52840 DK board using nrf Connect SDK 1.9.1
 * 
 **/

#include <stdlib.h>
#include <bluetooth/bluetooth.h>
#include <settings/settings.h>
#include <drivers/gpio.h>
#include <bluetooth/mesh.h>
#include <random/rand32.h>
#include <logging/log.h>
#include <drivers/led.h>
#include <device.h>
#include <errno.h>
#include <sys/util.h>
#include <zephyr.h>

// -------------------------------------------------------------------------------------------------------
// LED
// -------

// PWM for LEDs
LOG_MODULE_REGISTER(main, CONFIG_LOG_DEFAULT_LEVEL);

#if DT_NODE_HAS_STATUS(DT_INST(0, pwm_leds), okay)
#define LED_PWM_NODE_ID		DT_INST(0, pwm_leds)
#define LED_PWM_DEV_NAME	DEVICE_DT_NAME(LED_PWM_NODE_ID)
#else
#error "No LED PWM device found"
#endif

#define LED_PWM_LABEL(led_node_id) DT_PROP_OR(led_node_id, label, NULL),

const char *led_label[] = {
	DT_FOREACH_CHILD(LED_PWM_NODE_ID, LED_PWM_LABEL)
};

const int num_leds = ARRAY_SIZE(led_label);

const struct device *led_pwm;

//Brightness levels
#define MAX_BRIGHTNESS	100
#define MIN_BRIGHTNESS 0
#define LEVEL1_BRIGHTNESS 33
#define LEVEL2_BRIGHTNESS 66

#define FADE_DELAY_MS	10
#define FADE_DELAY	K_MSEC(FADE_DELAY_MS)

// states and state changes
uint8_t onoff_state;

uint8_t level = 0;

bool publish = false;
uint16_t reply_addr;
uint8_t reply_net_idx;
uint8_t reply_app_idx;

// usually set by the manufacturer - hard coded here for convenience

// device UUID
static uint8_t dev_uuid[16] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,0x00, 0x00, 0x00, 0x00,0x00, 0x00, 0x00, 0x00 };
void gen_uuid() {
    uint32_t rnd1 = sys_rand32_get();
    uint32_t rnd2 = sys_rand32_get();
    uint32_t rnd3 = sys_rand32_get();
    uint32_t rnd4 = sys_rand32_get();

    dev_uuid[15] = (rnd1 >> 24) & 0x0FF;
    dev_uuid[14] = (rnd1 >> 16) & 0x0FF;
    dev_uuid[13] = (rnd1 >>  8) & 0x0FF;
    dev_uuid[12] =  rnd1 & 0x0FF;

    dev_uuid[11] = (rnd2 >> 24) & 0x0FF;
    dev_uuid[10] = (rnd2 >> 16) & 0x0FF;
    dev_uuid[9] = (rnd2 >>  8) & 0x0FF;
    dev_uuid[8] =  rnd2 & 0x0FF;

    dev_uuid[7] = (rnd3 >> 24) & 0x0FF;
    dev_uuid[6] = (rnd3 >> 16) & 0x0FF;
    dev_uuid[5] = (rnd3 >>  8) & 0x0FF;
    dev_uuid[4] =  rnd3 & 0x0FF;

    dev_uuid[3] = (rnd4 >> 24) & 0x0FF;
    dev_uuid[2] = (rnd4 >> 16) & 0x0FF;
    dev_uuid[1] = (rnd4 >>  8) & 0x0FF;
    dev_uuid[0] =  rnd4 & 0x0FF;

    /* Set 4 MSB bits of time_hi_and_version field */
    dev_uuid[6] &= 0x0f;
    dev_uuid[6] |= 4 << 4;

    /* Set 2 MSB of clock_seq_hi_and_reserved to 10 */
    dev_uuid[8] &= 0x3f;
    dev_uuid[8] |= 0x80;

}

void ledsOn()
{
	for(int i=0;i<num_leds;i++){
		led_set_brightness(led_pwm, i, MAX_BRIGHTNESS);
	}
	level=MAX_BRIGHTNESS;
}

/**
 * @brief Turns the LEDs off
 * 
 */
void ledsOff()
{
	for(int i=0;i<num_leds;i++){
		led_set_brightness(led_pwm, i, MIN_BRIGHTNESS);
	}
	level=MIN_BRIGHTNESS;
}

/**
 * @brief Change the brightness of the LEDs according to the value in parameters
 * 
 * @param wantedValue 
 */
void ledsBrightnessSet(int wantedValue){
	uint8_t led;
	for (led = 0; led < num_leds; led++) {
		led_set_brightness(led_pwm, led, wantedValue);
	}
	level = wantedValue;
}

/**
 * @brief Display the current LEDs' brightness level 
 * 
 */
void getLedsBrightness(){
	printk("The brightness of the LEDs is %d%%\n",level);
}

/**
 * @brief The function takes the value received by the client and according to the current brightness level, increases or decreases it  
 * 
 * @param wantedState 
 */
void setWantedState(int wantedState){
	switch (wantedState)
	{
	case 0:
			switch(level){
			case MIN_BRIGHTNESS:
				printk("Already OFF\n");
				break;
			case LEVEL1_BRIGHTNESS:
				printk("Decreasing LEDs brightness");
				ledsBrightnessSet(MIN_BRIGHTNESS);
				break;
			case LEVEL2_BRIGHTNESS:
				printk("Decreasing LEDs brightness");
				ledsBrightnessSet(LEVEL1_BRIGHTNESS);
				break;
			case MAX_BRIGHTNESS:
				printk("Decreasing LEDs brightness");
				ledsBrightnessSet(LEVEL2_BRIGHTNESS);
				break;
			default:
				printk("Wrong Value\n");
			}
		break;
	
	case 1:
			switch(level){
			case MIN_BRIGHTNESS:
				printk("Increasing LEDs brightness");
				ledsBrightnessSet(LEVEL1_BRIGHTNESS);
				break;
			case LEVEL1_BRIGHTNESS:
				printk("Increasing LEDs brightness");
				ledsBrightnessSet(LEVEL2_BRIGHTNESS);
				break;
			case LEVEL2_BRIGHTNESS:
				printk("Increasing LEDs brightness");
				ledsBrightnessSet(MAX_BRIGHTNESS);
				break;
			case MAX_BRIGHTNESS:
				printk("Already at maximum brightness\n");
				break;
			default:
				printk("Wrong Value\n");
			}
		break;

	case 2:
		ledsOff();
		printk("Turning off LEDs");
		break;
	default:
		printk("Wrong Value\n");
		break;
	}
}


// provisioning callback functions
static void attention_on(struct bt_mesh_model *model)
{
	printk("attention_on()\n");
	ledsOn();
}

static void attention_off(struct bt_mesh_model *model)
{
	printk("attention_off()\n");
	ledsOff();
}

static const struct bt_mesh_health_srv_cb health_srv_cb = {
	.attn_on = attention_on,
	.attn_off = attention_off,
};

static int provisioning_output_pin(bt_mesh_output_action_t action, uint32_t number) {
	printk("OOB Number: %u\n", number);
	return 0;
}

static void provisioning_complete(uint16_t net_idx, uint16_t addr) {
    printk("Provisioning completed\n");
}

static void provisioning_reset(void)
{
	bt_mesh_prov_enable(BT_MESH_PROV_ADV | BT_MESH_PROV_GATT);
}


// provisioning properties and capabilities
static const struct bt_mesh_prov prov = {
	.uuid = dev_uuid,
	.output_size = 4,
	.output_actions = BT_MESH_DISPLAY_NUMBER,
	.output_number = provisioning_output_pin,
	.complete = provisioning_complete,
	.reset = provisioning_reset,
};


// generic onoff server message opcodes

	//message opcode definition : (an opcode (abbreviated from operation code, also known as instruction machine code) 
	//is the portion of a machine language instruction that specifies the operation to be performed.)
	/*
	We define constants for each of the message types that are part of the generic onoff client
	model and will be referencing them elsewhere in our code as we complete the node composition.
	*/
	#define BT_MESH_MODEL_OP_GENERIC_ONOFF_GET	BT_MESH_MODEL_OP_2(0x82, 0x01)
	#define BT_MESH_MODEL_OP_GENERIC_ONOFF_SET	BT_MESH_MODEL_OP_2(0x82, 0x02)
	#define BT_MESH_MODEL_OP_GENERIC_ONOFF_SET_UNACK	BT_MESH_MODEL_OP_2(0x82, 0x03)
	#define BT_MESH_MODEL_OP_GENERIC_ONOFF_STATUS	BT_MESH_MODEL_OP_2(0x82, 0x04)

// generic onoff server functions

	//need to forward declare as we have circular dependencies
	/**
	 * @brief The array of bt_mesh_model_op types maps the opcode of generic on off messages to functions
		which will handle messages of each type and indicates the minimum permitted access message
		payload length. BT_MESH_MODEL_END indicates the end of the definition.
	 * 
	 * @param publish 
	 * @param on_or_off 
	 */
	void generic_onoff_status(bool publish, uint8_t on_or_off);

	/**
	 * @brief set_onoff_state extracts the onoff state field in the received message and checks to see if it contains
		a different value to the model’s current on off state or not. If it does not then there is no state
		change being requested and so nothing to do.
		Otherwise, depending on the value of the onoff_state field, the function either switches the Thingy
		LED on or off. If an acknowledgement is required it calls a function called generic_onoff_status
	 * 
	 * @param model 
	 * @param ctx 
	 * @param buf 
	 * @param ack 
	 */
	static void set_onoff_state(struct bt_mesh_model *model, struct bt_mesh_msg_ctx *ctx, 
	struct net_buf_simple *buf, bool ack){
		uint8_t msg_onoff_state = net_buf_simple_pull_u8(buf);
		onoff_state = msg_onoff_state;
		uint8_t tid = net_buf_simple_pull_u8(buf);
		printk("set onoff state: onoff=%u TID=%u\n", onoff_state, tid);
		
		setWantedState(onoff_state);

		if(ack){
			generic_onoff_status(false, onoff_state);
		}
		/* If a server has a publish address, it is required to publish status on a state change*/
		if(model->pub->addr != BT_MESH_ADDR_UNASSIGNED){
			generic_onoff_status(true, onoff_state);
		}
	} 

	/**
	 * @brief The source address of the received generic on off get message, plus the associated application and
		network key indexes are saved in variables for use when responding with a status message. The
		function generic_onoff_status is called, indicating that a direct response message is required rather
		than publication of a status message to all subscribers.
	 * 
	 * @param model 
	 * @param ctx 
	 * @param buf 
	 */
	static void generic_onoff_get(struct bt_mesh_model *model, struct bt_mesh_msg_ctx *ctx, struct net_buf_simple *buf){
		printk("gen_onoff_get\n");
		// logged for interest only
		printk("ctx net_idx=0x%02x\n",ctx->net_idx);
		printk("ctx app_idx=0x%02x\n",ctx->app_idx);
		printk("ctx addr=0x%02x\n",ctx->addr);
		printk("ctx recv_dst=0x%02x\n",ctx->recv_dst);

		reply_addr = ctx->addr;
		reply_net_idx = ctx->net_idx;
		reply_app_idx = ctx->app_idx;
		generic_onoff_status(false, onoff_state);
		
	}
	static void generic_onoff_set(struct bt_mesh_model *model, struct bt_mesh_msg_ctx *ctx, struct net_buf_simple *buf){
		printk("gen_onoff_set\n");
		set_onoff_state(model, ctx, buf, true);

	}
	static void generic_onoff_set_unack(struct bt_mesh_model *model, struct bt_mesh_msg_ctx *ctx, struct net_buf_simple *buf){
		printk("generic_onoff_set_unack\n");
		set_onoff_state(model, ctx, buf, false);

	}

	static const struct bt_mesh_model_op generic_onoff_op[] = {
		{BT_MESH_MODEL_OP_GENERIC_ONOFF_GET, 0, generic_onoff_get},
		{BT_MESH_MODEL_OP_GENERIC_ONOFF_SET, 2, generic_onoff_set},
		{BT_MESH_MODEL_OP_GENERIC_ONOFF_SET_UNACK, 2, generic_onoff_set_unack},
		BT_MESH_MODEL_OP_END,	
	};
// generic onoff server model publication context
BT_MESH_MODEL_PUB_DEFINE(generic_onoff_pub, NULL, 2 + 1);

// -------------------------------------------------------------------------------------------------------
// Health Server
// -------------

BT_MESH_HEALTH_PUB_DEFINE(health_pub, 0);
static struct bt_mesh_health_srv health_srv = {
	.cb = &health_srv_cb,
};

// -------------------------------------------------------------------------------------------------------
// Composition
// -----------

// TODO complete the composition with the generic onoff server model and light HSL server model
static struct bt_mesh_model sig_models[] = {
		BT_MESH_MODEL_CFG_SRV,
		BT_MESH_MODEL_HEALTH_SRV(&health_srv, &health_pub),
		BT_MESH_MODEL(BT_MESH_MODEL_ID_GEN_ONOFF_SRV, generic_onoff_op, &generic_onoff_pub, NULL),
};

// node contains elements.note that BT_MESH_MODEL_NONE means "none of this type" ands here means "no vendor models"
static struct bt_mesh_elem elements[] = {
		BT_MESH_ELEM(0, sig_models, BT_MESH_MODEL_NONE),
};

// node
static const struct bt_mesh_comp comp = {
		.cid = 0xFFFF,
		.elem = elements,
		.elem_count = ARRAY_SIZE(elements),
};

// ----------------------------------------------------------------------------------------------------
// generic onoff status TX message producer
void generic_onoff_status(bool publish, uint8_t on_or_off){
		int err;
		struct bt_mesh_model *model = &sig_models[2];
			if (publish && model->pub->addr == BT_MESH_ADDR_UNASSIGNED) {
				printk("No publish address associated with the generic on off server model - add one with a configuration app\n");
				return;
			}

			if (publish) {
				struct net_buf_simple *msg = model->pub->msg;
				net_buf_simple_reset(msg);
				bt_mesh_model_msg_init(msg, BT_MESH_MODEL_OP_GENERIC_ONOFF_STATUS);
				net_buf_simple_add_u8(msg, level);
				printk("publishing on off status message\n");
				err = bt_mesh_model_publish(model);
				if (err) {
					printk("bt_mesh_model_publish err %d\n",err);
				}
			}else {
				uint8_t buflen = 7;
				NET_BUF_SIMPLE_DEFINE(msg, buflen);
				bt_mesh_model_msg_init(&msg, BT_MESH_MODEL_OP_GENERIC_ONOFF_STATUS);
				net_buf_simple_add_u8(&msg, level);
				struct bt_mesh_msg_ctx ctx = {
					.net_idx = reply_net_idx,
					.app_idx = reply_app_idx,
					.addr = reply_addr,
					.send_ttl = BT_MESH_TTL_DEFAULT,
				};
				printk("sending on off status message\n");
				if (bt_mesh_model_send(model, &ctx, &msg, NULL, NULL)){
					printk("Unable to send generic onoff status message\n");
				}
			}	
		getLedsBrightness();
	}

static void bt_ready(int err)
{
	if (err)
	{
		printk("bt_enable init failed with err %d\n", err);
		return;
	}
    printk("Bluetooth initialised OK\n");
	gen_uuid();
    printk("\n%02X%02X%02X%02X-%02X%02X-%02X%02X-%02X%02X-%02X%02X%02X%02X%02X%02X\n\n",
            dev_uuid[15], dev_uuid[14], dev_uuid[13], dev_uuid[12],dev_uuid[11], dev_uuid[10], dev_uuid[9], dev_uuid[8],
            dev_uuid[7], dev_uuid[6], dev_uuid[5], dev_uuid[4],dev_uuid[3], dev_uuid[2], dev_uuid[1], dev_uuid[0]);
	err = bt_mesh_init(&prov, &comp);

	if (err)
	{
		printk("bt_mesh_init failed with err %d\n", err);
		return;
	}

	printk("Mesh initialised OK\n");

	if (IS_ENABLED(CONFIG_SETTINGS)) {
		settings_load();
	    printk("Settings loaded\n");
	}

	/* This will be a no-op if settings_load() loaded provisioning info */
	/* run nrfjprog -e against your board (assuming it's a Nordic board) to clear provisioning data and start again */

     if (!bt_mesh_is_provisioned()) {
    	printk("Node has not been provisioned - beaconing\n");
		bt_mesh_prov_enable(BT_MESH_PROV_ADV | BT_MESH_PROV_GATT);
	} else {
    	printk("Node has already been provisioned\n");
	    printk("Node unicast address: 0x%04x\n",elements[0].addr);
	}

}

void configureLEDs(void){
	led_pwm = device_get_binding(LED_PWM_DEV_NAME);
	if (led_pwm) {
		LOG_INF("Found device %s", LED_PWM_DEV_NAME);
	} else {
		LOG_ERR("Device %s not found", LED_PWM_DEV_NAME);
		return;
	}

	if (!num_leds) {
		LOG_ERR("No LEDs found for %s", LED_PWM_DEV_NAME);
		return;
	}
	level = MIN_BRIGHTNESS;
	for(int i=0;i<num_leds;i++){
		led_set_brightness(led_pwm, i, level);
	}
}

void indicate_on() {
	ledsOn();
    k_sleep(K_MSEC(5000));
	ledsOff();	
}

void main(void)
{
	printk("NRF52840 Server (Lights) \n");

	configureLEDs();

    indicate_on();

	int err = bt_enable(bt_ready);
	if (err)
	{
		printk("bt_enable failed with err %d\n", err);
	}

}
