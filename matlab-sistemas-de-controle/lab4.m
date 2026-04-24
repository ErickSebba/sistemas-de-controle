A = 0.5;
Ke= 0.05;
Ks=0.015;
g= 9.8;
%ae0 = 0.6;
%as0=0.5;
%H0

ae0 = 0:0.01:1;
as0 = 0:0.01:1;
H0 = ((Ke .* ae0) ./ (Ks .* as0 .* sqrt(2*g))).^2;
plot3(as0,ae0,H0)
%surf(ae0, as0, H0)

%%

[AE0, AS0] = meshgrid(ae0, as0);

H0 = ((Ke .* AE0) ./ (Ks .* AS0 .* sqrt(2*g))).^2;
surf(AE0, AS0, H0)
xlabel('ae0')
ylabel('as0')
zlabel('H0')

%%
H0 = ((Ke .* ae0) ./ (Ks .* as0 .* sqrt(2*g))).^2;

mesh(ae0, as0,H0);
surf(ae0,as0,H0)
xlabel('ae0')
ylabel('as0')
zlabel('H0')
axis([0 1 0 1 0 1])