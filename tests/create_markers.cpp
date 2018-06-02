#include <iostream>
#include <opencv2/highgui/highgui.hpp>
#include <aruco/cameraparameters.h>
 #include <fstream>
using namespace std;



void create_big_markers(int start){
    stringstream cmd;cmd<<"aruco_create_marker "<<start<<" ou1.png 1000 ";
    system(cmd.str().c_str());
    stringstream cmd2;cmd2<<"aruco_create_marker "<<start+1<<" ou2.png 1000 ";
    system(cmd2.str().c_str());
    ofstream tex_file("file.tex");
    tex_file<<"\\documentclass[a4paper,10pt,onecolumn]{report}"<<endl;
    tex_file<<"\\usepackage{graphicx}"<<endl;
    tex_file<<"\\setlength{\\oddsidemargin}{-1.0cm}"<<endl;
    tex_file<<"\\setlength{\\textwidth}{18.5cm}"<<endl;
    tex_file<<"\\setlength{\\topmargin}{-3.7cm}"<<endl;
    tex_file<<"\\setlength{\\textheight}{38.5cm}"<<endl;
    tex_file<<"\\pagestyle{empty}"<<endl;
    tex_file<<"\\begin{document}"<<endl;
    tex_file<<"\\begin{figure}[h!]"<<endl;
    tex_file<<"\\centering"<<endl;
    tex_file<<"\\includegraphics[width=13.9cm]{ou1.png}"<<endl;
    tex_file<<"\\end{figure}"<<endl;
    tex_file<<"\\vspace{0.7cm}"<<endl;
    tex_file<<"\\begin{figure}[h!]"<<endl;
    tex_file<<"\\centering"<<endl;
    tex_file<<"\\includegraphics[width=13.9cm]{ou2.png}"<<endl;
    tex_file<<"\\end{figure}"<<endl;
    tex_file<<"\\end{document}"<<endl;
    stringstream cmd3;cmd3<<"pdflatex file.tex ";
    system(cmd3.str().c_str());

}


void create_small_markers(int start){


    vector<std::string> names;
    for(auto i=0;i<91;i++){
        int id=start+i*10;
        string name="out-"+to_string(id)+".png";
        stringstream cmd;cmd<<"aruco_create_marker "<<id<<" "<<name<<"  1000 1 ";
        system(cmd.str().c_str());
        names.push_back(name);
    }
    ofstream tex_file("file.tex");
    tex_file<<"\\documentclass[a4paper,10pt,onecolumn]{report}"<<endl;
    tex_file<<"\\usepackage{graphicx}"<<endl;
    tex_file<<"\\setlength{\\oddsidemargin}{-1.0cm}"<<endl;
    tex_file<<"\\setlength{\\textwidth}{16.5cm}"<<endl;
    tex_file<<"\\setlength{\\topmargin}{-2.5cm}"<<endl;
    tex_file<<"\\setlength{\\textheight}{38.5cm}"<<endl;
    tex_file<<"\\pagestyle{empty}"<<endl;
    tex_file<<"\\begin{document}"<<endl;


    int ncols=7;
    int nrows=1 + names.size()/ncols;
    int nimg=0;
    tex_file<<"\\begin{tabular}{|";
    for(int i=0;i<ncols;i++)tex_file<<"c|";
    tex_file<<"}";
    tex_file<<"\\hline"<<endl;

    for(int current=0;current<names.size();){
        for(int c=0;c<ncols && current<names.size();c++,current++){
            tex_file<<"\\includegraphics[width=2cm]{"<<names[current]<<"} "<<endl;
            if (c!=ncols-1) tex_file<<" & ";
        }
        tex_file<<"\\\\ \\hline"<<endl;
    }
    tex_file<<"\\end{tabular}"<<endl;
    tex_file<<"\\end{document}"<<endl;

    //    if( (++nimg)%10==0)     tex_file<<"\\newpage"<<endl;

    stringstream cmd3;cmd3<<"pdflatex file.tex ";
    system(cmd3.str().c_str());

}


int main(int argc,char **argv){
    if(argc!=2){cerr<<"USage: start_id"<<endl;return -1;}
    create_big_markers (atoi(argv[1]));

    system("evince  file.pdf&");
}
