//
// Created by hoseok on 10/14/18.
//

#include "print_graph.h"

void PhysExam::printString(FILE *out, std::string str){
    fprintf(out,"%s\n",str.c_str());
}
void PhysExam::printCoord(FILE *out, double x,double y){
    fprintf(out,"%lf %lf\n",x, y);
}