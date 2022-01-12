#include <iostream>
#include <algorithm>
#include "math.h"

#define max_len 1500
#define _USE_MATH_DEFINES

// -------------------------------------------------- PARAMETERS --------------------------------------------------------

// j è il segmento
// jj sono i valori dentro il segmento (sottosegmento)

// input parameters
bool bEna = true;
bool bRndOn = true;
float ehr_sVeh_V[max_len];  // segment
float ehr_vVeh_V[max_len];  // speed
float ehr_percSlop_V[max_len];  // slope (-100)
float ehr_sStopEvent_V[max_len];  // stop_position
float ehr_typeStopEvent_V[max_len];  // stop_event
float ehr_sTraffic[max_len];  // traffic_segments
float ehr_typeTraffic[max_len];  // traffic_codes

// TODO debug: import data from Excel file 20210914_S1_BO_Congested.xlsx
/*
df = pd.ExcelFile('20210914_S1_BO_Congested.xlsx').parse('NavData')

ehr_sVeh_V = list(map(float, df['segment'].tolist()))
ehr_vVeh_V = list(map(float, df['speed'].tolist()))
ehr_percSlop_V = list(map(float, df['slope'].tolist()))
ehr_sStopEvent_V = list(map(float, df['stop_position'].tolist()))
ehr_typeStopEvent_V = list(map(float, df['stop_event'].tolist()))
ehr_sTraffic = list(map(float, df['traffic_segments'].tolist()))
ehr_typeTraffic = list(map(float, df['traffic_codes'].tolist()))


ehr_sVeh_V = [x for x in ehr_sVeh_V if not math.isnan(x)]
ehr_vVeh_V = [x for x in ehr_vVeh_V if not math.isnan(x)]
ehr_percSlop_V = [x for x in ehr_percSlop_V if not math.isnan(x)]
ehr_sStopEvent_V = [x for x in ehr_sStopEvent_V if not math.isnan(x)]
ehr_typeStopEvent_V = [x for x in ehr_typeStopEvent_V if not math.isnan(x)]
ehr_sTraffic = [x for x in ehr_sTraffic if not math.isnan(x)]
ehr_typeTraffic = [x for x in ehr_typeTraffic if not math.isnan(x)]

/**/

// select driver behavior
int spp_stDriver_C = 2;  // 1 = quiet / 2 = normal / 3 = aggressive
int startVel_0 = 0;

// output parameters
float velocities[max_len];
float distance[max_len];
float trueStopEvents[max_len];
float slope[max_len];

// variables

// (1) part
float acc, maxLongDec, K, K1 = 0;
int VUTMaxLongDec = 2;   // Max deceleration
int VUTMaxLongAcc = 3;   // Max acceleration

// (2) part
int ehr_sizeMemory_C = 1500;

// (3) part

// (4) part
float MaxAlwSpeed_kmph[max_len];
float nodes[max_len];
float codes[max_len];
float WMAS[max_len];

int spp_stGreen_C = 1;
int spp_stOrange_C = 2;
int spp_stRed_C = 3;
int spp_stBlack_C = 4;

float spp_rGreen_C = 0.85;
float spp_rOrange_C = 0.6;
float spp_rRed_C = 0.4;
float spp_rBlack_C = 0.3;

// (5) part
int c = 0;

// (6) part
float length[max_len];
float v_endVel[max_len];
int jj = 0;
float MaxAlwSpeed_mps[max_len];

// (7) part
float k, Q1, l1 = 0;
float vel[0] * ehr_sizeMemory_C;

// (8) part
float pi = M_PI;
float MAG1, MAG2, MAG3, MAG4 = 0;
float FRQ1, FRQ2, FRQ3, FRQ4 = 0;

// (9) part
int y = 0;

int main() {
    if(!bRndOn){
        //random.seed(1) in python
        srand(1);
    }

    // (1) driver selection

    switch(spp_stDriver_C){
        case 1:
            acc = VUTMaxLongAcc * 0.30;
            maxLongDec = VUTMaxLongDec * 0.40;
            K = 1/1000;
            K1 = 1/1000;
            break;

        case 2:
            acc = VUTMaxLongAcc * 0.80;
            maxLongDec = VUTMaxLongDec * 0.30;
            K = 1 / 60;
            K1 = 10;
            break;

        case 3:
            acc = VUTMaxLongAcc * 0.90;
            maxLongDec = VUTMaxLongDec * 0.70;
            K = 1 / 10;
            K1 = 1 / 10;
            break;
    }
    int n = sizeof (ehr_sVeh_V) / sizeof(ehr_sVeh_V[0]);
    float totalLength = *std::max_element(ehr_sVeh_V, ehr_sVeh_V + n);
    int cumL = 0;

    // (2) discretization interval
    // we have 1500 elements and this limit is due to the fac that the hw has this limit
    // we must fit the path on these 1500 elements
    // distance / 1500 = discretization
    // every time that the difference between the one before and the one after i create a new segment with a different
    // discretization (disc0)
    // 0.6 is used in order to avoid overflow because of 3+1 segments

    float disc0 = totalLength / (ehr_sizeMemory_C * 0.60);

    // (3) nodes creation = union space vectors (sorting: increasing)
    // creazione asse x
    // i nodes sono i punti messi insieme tra gli input. mi arrivano 4 coppie di vettori
    // creo un unico vettore con tutti i segmenti (unico vettore spazio) e inserisco dentro tutto

    int size_ehr_sVeh_V = sizeof(ehr_sVeh_V) / sizeof(ehr_sVeh_V[0]);
    int size_ehr_sStopEvent_V = sizeof(ehr_sStopEvent_V) / sizeof(ehr_sStopEvent_V[0]);
    int size_ehr_sTraffic = sizeof(ehr_sTraffic) / sizeof(ehr_sTraffic[0]);

    float nodes_temp [size_ehr_sVeh_V + size_ehr_sStopEvent_V];
    int size_nodes_temp = sizeof(nodes_temp) / sizeof(nodes_temp[0]);
    float nodes [size_nodes_temp + size_ehr_sTraffic];

    std::copy(ehr_sStopEvent_V, ehr_sStopEvent_V + size_ehr_sStopEvent_V, std::copy(ehr_sVeh_V, ehr_sVeh_V + size_ehr_sVeh_V, nodes_temp));
    std::copy(ehr_sTraffic, ehr_sTraffic + size_ehr_sTraffic, std::copy(nodes_temp, nodes_temp + size_nodes_temp, nodes));

    // (4) speed limit and traffic code assegnation to every segment of nodes array
    // creazione asse y

    int i, j = 0;
    while(i < (sizeof(nodes)/sizeof(nodes[0])) - 1) {
        if(nodes[i] <= ehr_sVeh_V[j]) {
            MaxAlwSpeed_kmph[i] = ehr_vVeh_V[j];
            i += 1;
        }
        else {
            j += 1;
            MaxAlwSpeed_kmph[i] = ehr_vVeh_V[j];
            i += 1;
        }
    }
    i, j = 0;
    // TODO da rivedere
    if(ehr_typeTraffic[j] == 0){
        for(int index = 0; index < (sizeof(MaxAlwSpeed_mps)/ sizeof(MaxAlwSpeed_mps[0])); index++){
            MaxAlwSpeed_mps.append(MaxAlwSpeed_kmph[index] * (0.95 / 3.6));
        }
    }
    else{
        while i < len(nodes) - 1{
            if nodes[i] <= ehr_sTraffic[j]{
                codes.append(ehr_typeTraffic[j])
                i += 1
            }
            else{
                j += 1
                codes.append(ehr_typeTraffic[j])
                i += 1
            }
        }
        i = 0
        while i < len(codes) - 1
        {
            if(codes[i] == spp_stGreen_C){
            codes[i] = spp_rGreen_C;
            i += 1;
            }
            else if(codes[i] == spp_stOrange_C){
                codes[i] = spp_rOrange_C;
                i += 1;
            }
            else if(codes[i] == spp_stRed_C){
                codes[i] = spp_rRed_C;
                i += 1;
            }
            else if(codes[i] == spp_stBlack_C){
                codes[i] = spp_rBlack_C;
                i += 1;
            }
            else{
                codes[i] = spp_rGreen_C;
            }
        }
        // TODO
        for index1 in range(0, len(codes) - 1):
            WMAS.append(MaxAlwSpeed_kmph[index1] * codes[index1])
            MaxAlwSpeed_mps = [x / 3.6 for x in WMAS]
    }

    // (5) creation of 4 segments between a node and the other (if two nodes are too close)

    j, c = 0;
    i = j + 1;
    while j <= len(nodes) - 2{
        if(j == 0){  // inizializzazione
            l = nodes[j];
        }
        else {
            l = nodes[j] - nodes[j - 1];
        }

        if(l <= 4 * disc0){  // creazione segmenti
            n = 5
            disc = l / n
        }
        else {
            disc = disc0
            n = math.ceil(l / disc)
        }
        if(n > ehr_sizeMemory_C) {
            n = ehr_sizeMemory_C
        }
        // if nodes[j + 1] != 0:
        if(j < len(nodes) - 2){
            l1 = nodes[j + 1] - nodes[j];
            if(l1 <= 4 * disc0){
                n1 = 5;
                disc1 = l1 / n1;
                }
            else {
                disc1 = disc0;
                n1 = math.ceil(l1 / disc1);
            }
        }

        // (6) 3 conditions:
        //     1. starting segment: it assigns init speed ???
        //     2. final segment: it assigns final speed ???
        //     3. intermediate segment



    return 0;
}