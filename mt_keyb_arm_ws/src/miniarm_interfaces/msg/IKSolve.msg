miniarm_interfaces/IKTarget target      #The target position

float32 angle_full_1                    #Angle of the first joint (uncapped)
float32 angle_full_2                    #Angle of the second joint (uncapped)

bool click

float32 angle_1                         #Angle of the first joint (capped)
float32 angle_2                         #Angle of the second joint (capped)
miniarm_interfaces/IKTarget reached     #The actual position reched due to physical limitations
float32 dst                             #Distance between the reached position and the target position