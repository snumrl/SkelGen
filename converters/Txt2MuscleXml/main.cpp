#include <iostream>
#include <stdio.h>
#include <string.h>
FILE *in=fopen("../input.txt","r"),*out=fopen("../output.txt","w");
using namespace std;

char pelvis[3][20] = {"Coccyx", "SacrumShape", "L_Os_CoxaeShape"};
char torso[30][20]={"Atlas",
                  "Axis",
                  "Vertebrae_C03",
                  "Vertebrae_C04",
                  "Vertebrae_C05",
                  "Vertebrae_C06",
                  "Vertebrae_C07",
                  "Vertebrae_T01",
                  "Vertebrae_T02",
                  "Vertebrae_T03",
                  "Vertebrae_T04",
                  "Vertebrae_T05",
                  "Vertebrae_T06",
                  "Vertebrae_T07",
                  "Vertebrae_T08",
                  "Vertebrae_T09",
                  "Vertebrae_T10",
                  "Vertebrae_T11",
                  "Vertebrae_T12",
                  "Vertebrae_L01",
                  "Vertebrae_L02",
                  "Vertebrae_L03",
                  "Vertebrae_L04"};
char tibia[5][20]={"L_Patella"};
char foot[30][40]={"L_Lateral_Cuneiform",
                   "L_Intermediate_Cuneiform",
                   "L_F_Distal_Phalanx_2",
                   "L_F_Distal_Phalanx_4",
                   "L_F_Distal_Phalanx_3",
                   "L_F_Distal_Phalanx_5",
                   "L_F_Middle_Phalanx_5",
                   "L_F_Middle_Phalanx_4",
                   "L_F_Middle_Phalanx_3",
                   "L_F_Middle_Phalanx_2",
                   "L_F_Proximal_Phalanx_4",
                   "L_F_Proximal_Phalanx_3",
                   "L_F_Proximal_Phalanx_2",
                   "L_F_Metatarsal_1",
                   "L_F_Distal_Phalanx_1",
                   "L_F_Proximal_Phalanx_1",
                   "L_F_Proximal_Phalanx_5",
                   "L_F_Metatarsal_2",
                   "L_F_Metatarsal_3",
                   "L_F_Metatarsal_4",
                   "L_F_Metatarsal_5",
                   "L_Medial_Cuneiform",
                   "L_Cuboid",
                   "L_Navicular",
                   "L_Talus",
                   "L_Calcaneus"};

char type[55], name[55], attach_name[55];

char memN[55][55];
double memV[55];

char* convert(char* name){
    for (int i=0;i<3;i++){
        if (strcmp(pelvis[i], name)==0) return "Pelvis";
    }

    for (int i=0;i<23;i++){
        if (strcmp(torso[i], name)==0) return "Spine";
    }

    for (int i=0;i<1;i++){
        if (strcmp(tibia[i], name)==0) return "L_Tibia";
    }

    for (int i=0;i<26;i++){
        if (strcmp(foot[i], name)==0) return "L_Foot";
    }
    return name;
}
int main() {
    fprintf(out,"<Muscle>\n");
    double x,y,z;
    fscanf(in,"%s",type);
    int flag=0;
    for (;;){
        fscanf(in,"%s",name);
        int cnt=0;
        fprintf(out,"\t<Unit name=\"%s\" f0=\"2000.000000\" lm=\"1.000000\" lt=\"0.200000\" pen_angle=\"0.000000\">\n", convert(name));
        for (;;) {
            if (fscanf(in, "%s", type) == EOF) {
                flag = 1;
                break;
            }
            if (type[0] == 'u') break;
            fscanf(in, "%s %lf %lf %lf\n", attach_name, &x, &y, &z);
            fprintf(out, "\t\t<Waypoint body=\"%s\" p=\"%lf %lf %lf \" />\n", convert(attach_name), x, y, z);
            strcpy(memN[cnt],convert(attach_name));
            memV[cnt*3+0]=x;
            memV[cnt*3+1]=y;
            memV[cnt*3+2]=z;
            cnt++;
        }
        fprintf(out,"\t</Unit>\n");

        if (name[0]!='L') continue;
        name[0]='R';
        fprintf(out,"\t<Unit name=\"%s\" f0=\"2000.000000\" lm=\"1.000000\" lt=\"0.200000\" pen_angle=\"0.000000\">\n", convert(name));
        for (int i=0;i<cnt;i++){
            char temp[55];
            strcpy(temp,memN[i]);
            if(temp[0]=='L') temp[0]='R';
            fprintf(out, "\t\t<Waypoint body=\"%s\" p=\"%lf %lf %lf \" />\n", temp, -memV[i*3], memV[i*3+1], memV[i*3+2]);
        }
        fprintf(out,"\t</Unit>\n");
        if (flag==1) break;
    }
    fprintf(out,"</Muscle>\n");
    return 0;
}
