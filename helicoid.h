#ifndef HELICOID_H
#define HELICOID_H

#include "hittable.h"
#include "vec3.h"
#include "math.h"
#define PI 3.14159265

class helicoid : public hittable {
    public:
        helicoid() {}
        helicoid(point3 orig, double r_in, double r_out, double h, double a, double res, shared_ptr<material> m)
            : origin(orig), radius_in(r_in), radius_out(r_out), height(h), coef(a), resolution(res), mat_ptr(m) {};

        helicoid(point3 orig, double r_in, double r_out, double h, double a, double res) : origin(orig), radius_in(r_in), radius_out(r_out), height(h), coef(a), resolution(res) {};

        virtual bool hit(
            const ray& r, double t_min, double t_max, hit_record& rec) const override;

    public:
        point3 origin;
        double coef;
        double radius_in;
        double radius_out;
        double height;
        double resolution;
        shared_ptr<material> mat_ptr;
};

bool helicoid::hit(const ray& r, double t_min, double t_max, hit_record& rec) const {
    // define control variables
    int state = 0;
    double dist_to_origin = infinity;
    double current_dist;
    bool is_out = true;
    bool check_1;
    bool check_2;
    vec3 v0;
    vec3 v1;
    vec3 v2;
    vec3 p_ref;
    double denom;
    double t;
    vec3 v_ref0;
    vec3 v_ref1;
    vec3 v_ref2;

    // Normals
    vec3 n0;
    vec3 n1;
    vec3 n2;
    vec3 N;

    //define first 2 triangle points
    vec3 p_0 = vec3(radius_out * cos(0), 0, radius_out * sin(0)) + origin;
    vec3 p_1 = vec3(radius_in * cos(0), 0, radius_in * sin(0)) + origin;
    vec3 p_2;
    auto hit_found = false;

    for (double teta = resolution; teta <= (2 * PI * height + resolution/2); teta += resolution) {
        
        // Precisa rodar 2 vezes para o mesmo teta ( Usando raio in e outra usando raio out)
        for (int aux=0; aux < 2; aux++){
            if(state == 0){
                // Cálcula o próximo ponto
                if(is_out){
                    p_2 = vec3(radius_out * cos(teta), coef/PI * teta, radius_out * sin(teta)) + origin;
                    is_out = false;
                }
                else{
                    p_2 = vec3( radius_in * cos(teta), coef/PI * teta, radius_in * sin(teta)) + origin;
                    is_out = true;
                }
                state = 1;
            }

            if(state == 1){

                // Cálcula a normal
                N = cross((p_1-p_0),(p_2-p_0));
                N = unit_vector(N);

                // Checa se o raio esta paralelo á superficie
                denom = dot(N,r.direction());
                if(abs(denom)<=0.001){
                    state = -1; 
                }
                else{
                    state = 2;
                }
            }

            if(state == 2){
                // Descobre em qual ponto o raio intersecta com plano do triângulo
                t = dot(N,p_0-r.origin())/denom;

                // Checa se está dentro do range esperado
                if (t < t_min || t_max < t) {
                    state = -1;
                }
                else{
                    state = 3;
                    p_ref = r.at(t);
                }
            }

            if(state ==3){
                current_dist = (r.origin()-p_ref).length();
                if (current_dist > dist_to_origin){
                    // Caso o ponto de intersecção esteja mais longe que o último ponto, considera que o triângulo está atrás
                    // de outro triângulo
                    state = -1;
                }
                else{
                    // Checa se o ponto de intersecção está dentro do triângulo
                    v0 = p_1-p_0;
                    v1 = p_2-p_1;
                    v2 = p_0-p_2;

                    v_ref0 = p_ref - p_0;
                    v_ref1 = p_ref - p_1;
                    v_ref2 = p_ref - p_2;

                    // Normals
                    n0 = cross(v0, v_ref0);
                    n1 = cross(v1, v_ref1);
                    n2 = cross(v2, v_ref2);

                    // Checa se todos vetores normais estão na mesma direção
                    check_1 = dot(n0,n1)>=0;
                    check_2 = dot(n0,n2)>=0;

                    if(check_1 && check_2){

                        state = 4;
                    }
                    else{
                        state = -1;
                    }
                }
            }

            if(state == 4){
                if(dot(N,r.dir)>0){
                    N = N*-1;
                }
                
                // Estado caso o raio tenha intersecionado
                rec.t = t;
                rec.p = r.at(rec.t);
                rec.set_face_normal(r, N);
                rec.mat_ptr = mat_ptr;

                state = -1;
                dist_to_origin = current_dist;
                hit_found = true;
            }

            if(state == -1){
                // Recomeça o ciclo
                p_0 = p_1;
                p_1 = p_2;
                state = 0;
            }
        }
    }
    return hit_found;
}
#endif