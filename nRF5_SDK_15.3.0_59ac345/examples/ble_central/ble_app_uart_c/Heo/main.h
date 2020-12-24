//
// Header
//

#define BLE_EVT_COUNT 3

static uint8_t temp_array[BLE_NUS_MAX_DATA_LEN];
static uint8_t* temp_current = temp_array;
static uint8_t MASTER_MAP[BLE_EVT_COUNT] = {0, };

APP_TIMER_DEF(m_lte_timer_id);


// From the Board to the LTE modem
static bool at_send(uint8_t * data, uint16_t size);

// 
uint32_t checkMsg(char* src, char* pattern, int time, bool qcds_check, int length);

uint32_t checkMsgMQTT(char* src, char* pattern, int time, int length);

void lte_check(void);

void lte_setup(void);

void sendlte(uint8_t* data);

static void lte_timer_handler(void * p_context);
static void create_timer(void);
static void start_lte_timer(void);

void setmap(uint8_t (*map), int num);
void clearmap(uint8_t (*map), int num);
void resetmap(uint8_t (*map));
int checkmap(uint8_t (*map), int event);

void rptevt(uint8_t (*map));