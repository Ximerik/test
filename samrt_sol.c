

struct event{ // dicrete struct
	uint8_t what,	// comand
			where;	// place
	struct time _clock;
	struct event* left;
	struct event* right;
};

struct event create_new_event(void){
	return (struct event*) malloc(sizeof(struct event));
}

// for the first time without limits check
struct event* add_new_event(uint8_t comand, uint8_t place, uint8_t _minute, uint8_t _hour, uint8_t day, struct event *event_pointer){
	struct temp_time{ // temp struct for compare
		unsigned second  :8;
		unsigned minute  :8;
		unsigned hour	 :8;
		unsigned day	 :4;
	} temp_clock;
	temp_clock.second 	= 0;
	temp_clock.minute 	= _minute;
	temp_clock.hour 	= _hour;
	temp_clock.day 		= _day;
	if(event_pointer==NULL){
		event_pointer = create_new_event(void);
		event_pointer->what 			= comand;
		event_pointer->where 			= place;
		event_pointer->_clock-> minute 	= _minute;
		event_pointer->_clock->hour    	= _hour;
		event_pointer->_clock->day 	   	= _day;
		event_pointer->left = event_pointer->right = NULL;
	else if ((event_pointer->_clock == temp_clock) || (event_pointer->_clock < temp_clock)){
		event_pointer->left = add_new_event(comand, place, _minute, _hour, _day, event_pointer->left);
	else
		event_pointer->right = add_new_event(comand, place, _minute, _hour, _day, event_pointer->right);
		return event_pointer;
		}
	}
}



/* REAL TIME CLOCK MODULE */
struct time{ // dinamic struct
	unsigned second  :8;
	unsigned minute  :8;
	unsigned hour	 :8;
	unsigned day	 :4;
} _clock;


/*CLOCK FREQ 32768*/
void clock(void){ // calls on timer overflow interupt
	_clock.second++;
		if (_clock.second > 59){
			_clock.second = 0;
			_clock.minute++;
				if (_clock.minute > 59){
					_clock.minute = 0;
					_clock.hour++;
						if (_clock.hour > 23){
							_clock.hour = 0;
							_clock.day++;
								if (_clock.day > 6)
									_clock.day = 0;
						}
				}
		}		
}

