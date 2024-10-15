#include <Arduino.h>
#include "sensors/BME680/BME680Sensor.h"
#include "sensors/BNO055/BNO055Sensor.h"
#include "utils/logger/CborLogger.h"

BME680Sensor bme680;
BNO055Sensor bno055;
CborLogger cbor_logger;



typedef struct node {
    float number;
    struct node *next;
}
node;

node* zeros(int n) {
    node* list = (struct node*) malloc(sizeof(node));
    node* tmp = list;
    list->number = 0;
    for (int i = n - 1; i > 0; i--) {
        node* neew = (struct node*) malloc(sizeof(node));
        neew->number = 0;
        tmp->next = neew;
        tmp = tmp->next;
    }
    tmp->next = NULL;
    return list;
}

int len(node *list) {
    int n = 0;
    for(; list != NULL; list = list->next) {
        n++;
    }
    return n;
}

void push(node *list, float element) {
    node *n = (struct node*) malloc(sizeof(node));
    n->number = element;
    n->next = NULL;
    float old_num = list->number;
    list->number = element;
    n->next = list->next;
    n->number = old_num;
    list->next = n;
    return;
}

void pop(node *list) {
    if (list == NULL) {return;}
    while(list->next->next != NULL)
    {
        list = list->next;
    }
    free(list->next);
    list->next = NULL;
}

float mean_n(node* list, int start, int end) {
    int l = len(list);
    if (start < 0) {start = start + l;}
    if (end < 0) {end = end + l;}
    if (list == NULL) {
        return 0;
    }
    float sum = 0;
    int n = 0;
    for (;;) {
        if (n >= start && n <= end) {sum = sum + list->number;}
        list = list->next;
        if (list == NULL) {break;}
        n++;
    }
    return sum/(float)(end - start + 1);
}


void send_telemetry(node *acc, node *pres) {

}


bool check_sensors() {
    return true;
}

bool get_start_command() {
    return true;
}

bool get_launch_command() {
    return true;
}

bool get_abort_command() {
    return false;
}

void command_ignition() {}
void deploy_chute() {}


const int update_period_ms = 25;
const int take_off_acc = 30;
const int pressure_delta = 10;

class Rocket {
    private:
        int state;
        sensors_vec_t acc_vect;
        float curr_acc;
        uint32_t curr_press;
        uint32_t min_press;
        node *acceleration;
        node *pressure;
        int countdown;

        void state0() {
            // initial state
            if (get_start_command()) {
                state = 1;
            }
            delay(1000 - update_period_ms);
        }

        void state1() {
            // check_state
            if (check_sensors() && get_launch_command()) {
                state = 2;
            }
            delay(1000 - update_period_ms);            
        }

        void state2() {
            // countdown and igition
            if (countdown > 0) {
                if (!check_sensors() || get_abort_command()) {
                    state = 0;
                }
                countdown--;
            } else {
                command_ignition();
                int max_ignition_time = 5000;
                while (mean_n(acceleration, 0, 4) < take_off_acc) {
                    delay(25);
                    max_ignition_time = max_ignition_time - 25;
                    if (max_ignition_time < 0) {
                        state = 0;
                        break;
                    }
                }
                state = 3;
            }
        }

        void state3() {
            // powered and unpowered ascent
            min_press = min(curr_press, min_press);
            if (mean_n(acceleration, 0, 5) < 10 && curr_press > min_press + pressure_delta) {
                deploy_chute();
                state = 4;
            }
        }

        void state4() {
            // descent
            if (abs(mean_n(pressure, 0, 5) - curr_press) < 5) {
                state = 5;
            }
        }
        
        void state5() {
            // recovery
        }

        float get_mod_acc() {
            acc_vect = bno055.getData().getAccelerometer();
            return pow(pow(acc_vect.x, 2) + pow(acc_vect.y, 2) + pow(acc_vect.z, 2), 0.5);
        }

        uint32_t get_press() {
            return bme680.getData().getPressure();
        }

    public:
        Rocket() {
            state = 0;
            countdown = 30;
            acceleration = zeros(1);
            pressure = zeros(1);
            for (int i = 0; i < 1000/update_period_ms; i++) {
                push(acceleration, get_mod_acc());
                push(pressure, get_press());
                delay(update_period_ms);
            }
            update();
            min_press = curr_press;
        }

        void update() {
            bme680.readData();
            bno055.readData();
            curr_acc = get_mod_acc();
            curr_press = get_press();
            push(acceleration, curr_acc);
            push(pressure, curr_press);

            send_telemetry(acceleration, pressure);

            if (len(acceleration) > 8) {
                pop(acceleration);
                pop(pressure);
            }
            
            switch (state) {
                case 0: 
                    state0();
                    break;
                case 1:
                    state1();
                    break;
                case 2:
                    state2();
                    break;
                case 3:
                    state3();
                    break;
                case 4:
                    state4();
                    break;
                case 5:
                    state5();
                    break;
            };
        }
};

void setup()
{
    bme680.init(BME680_I2C_ADDR_1);
    bno055.init();
}

Rocket rocket;

void loop()
{
    rocket.update();
    delay(update_period_ms);
}