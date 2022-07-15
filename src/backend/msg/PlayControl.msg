int8 action_type # 1=pause/play, 2=forward/backward, 3=screenshot, 4=Set 0>%<1
string action_payload
#payload 1: {pause=bool}
#payload 2: {offset=int}
#payload 3: {}
#payload 4: {percent=float}