#ifndef JOINT_DATA_TYPE_H
#define JOINT_DATA_TYPE_H
typedef struct{
    unsigned int joint_pos_raw[16];
    unsigned int joint_pos_abs[16];
    unsigned int joint_force[16];
    bool joint_pos_valid[16];
    bool joint_online[16];

}JOINT_DAT_TYPE;
#endif // JOINT_DATA_TYPE_H
