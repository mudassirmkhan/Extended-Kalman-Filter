disp('assignment 1')
Data = csvread('calibration.txt'); % Reading file
 
T = Data(:,1); % Time
r_t = Data(:,2); % Right ticks
l_t = Data(:,3);% Left ticks
x=Data(:,4);% north gps
y=Data(:,5);% east gps
mag_x=Data(:,6);% Magnetometer x-Measurements in gauss
mag_y=Data(:,7);% Magnetometer y-Measurements in gauss
mag_z=Data(:,8);% Magnetometer z-Measurements in gauss 
gyro_z=Data(:,9);% Gyroscope z-axis Measurements
 
 
Outdoor = csvread('Outdoor.txt'); % Reading file
T1 = Outdoor(:,1); % Time
r_t1= Outdoor(:,2); % Right ticks
l_t1 = Outdoor(:,3);% Left ticks
n_gps1=Outdoor(:,4);% north gps
e_gps1=Outdoor(:,5);% east gps
mag_x1=Outdoor(:,6);% Magnetometer x-Measurements in gauss
mag_y1=Outdoor(:,7);% Magnetometer y-Measurements in gauss
mag_z1=Outdoor(:,8);% Magnetometer z-Measurements in gauss 
gyro_z1=Outdoor(:,9);% Gyroscope z-axis Measurements
 
 
 
 
% 
dia =0.1;
circumference=pi*dia;
 vR=(r_t/360)*circumference;
% vL=(l_t/360)*circumference;
 v=(0.5*(vR+vL));
e_t=600;%encoder ticks
distance_travelled=(e_t/360)*circumference;
 
 
size(gyro_z);
size(gyro_z1);
size(T1);
size(mag_x);
size(mag_y);
size(mag_y1);
k=1;
error_gyro=zeros(319,1);
for i=1:319
    if l_t==0 && r_t==0
        error_gyro(i)=0;
    else
       error_gyro(i)=gyro_z(i); 
    end
    k=k+1;
end
bias_gyro=mean(error_gyro);
 
for d=1:5224
  
    gyro_z1(k)=(gyro_z(k))-(bias_gyro);
end
%for calculating theeta
Q=zeros(319,1);
F=zeros(319,1);
for m=1:319
    Q(m)=mag_y(m)/mag_x(m);
    F(m)=atan(Q(m));
end
 
time_diff=zeros(319,1);
error_mag=zeros(319,1);
actual_angle=zeros(319,1);
for r=1:318
time_diff(r)=T(r+1)-(T(r));
actual_angle(r)=F(r+1)-(gyro_z(r).*time_diff(r));
error_mag(r)=actual_angle(r)-(F(r));
end
bias_mag=mean(error_mag);
S=zeros(5224,1);
G=zeros(5224,1);
U=zeros(5224,1);
for j=1:5224
 
    S(j)=mag_y1(j)/mag_x1(j);
    G(j)=atan(S(j));
    U(j)=G(j)-bias_mag;
  
end
L=mean(U);
C=[1 0 0;0 1 0;0 0 1];
D=transpose(C);
Q=[1 0;0 1];
R=[1 0 0;0 1 0;0 0 0.1];
%taking theeta as a here
x_=zeros(5224,1);
y_=zeros(5224,1);
U_=zeros(5224,1);
P_=zeros(5224,1);
E_=zeros(5224,1);
K_=zeros(5224,1);
x_new=zeros(5224,1);
U_new=zeros(5224,1);
y_new=zeros(5224,1);
output=zeros(5224,1);
h=zeros(3,1);
P=zeros(3,1);
S= -v*sin(L);
N=v*cos(L);
for i=1:5224
    A=[0 0 S;0 0 N;0 0 0];
B=transpose(A);
% Prediction phase 
output is taken as out
output(i)=[n_gps1(i),e_gps1(i),U(i)];
x_(T1(i))=x(T1(i)-1)+v*cos(U(i))*(T1(i)-T1(i-1));
y_(T1(i))=y(T1(i)-1)+v*sin(U(i))*(T1(i)-T1(i-1));
U_(T1(i))=U(T(i)-1)+gyro_z1*(T1(i)-T1(i-1));
%correction phase
P_(i)=(A(i).*P)+(B(i).*P)+Q;
E_(i)=D.*C.*P_;
K_(I)=D.*P_*(R+E)^(-1);
x_new(i)=x_(T(i))+K_(i)*(output-h(x_(T(i))));
y_new(i)=y_(T(i))+K_(i)*(output-h(y_(T(i))));
U_new(i)=U_(T(i))+K_(i)*(output-h(U_(T(i))));
output(i)=[0 0 0];
P(i)=(1-K(i).*C)^(-1);
end
 
plot(x_new,y_new)
plot(n_gps1,e_gps1)
