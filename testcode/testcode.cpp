#include<stduio.h>
float now, befor;
float dt;
int main(){
    befor = 0;
    now = 10;
    dt = now - befor;
    printf(dt);
    return 0;
}