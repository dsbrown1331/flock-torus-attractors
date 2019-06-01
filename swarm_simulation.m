clear all
close all

x=1;
y=2;

%set task to one of these
FLOCK = 0;  %initialize to form a flock
TORUS = 1;  %initialize to form a torus
RANDOM = 3; %initialize randomly

task = TORUS;



%agent parameters
Ns = [100]; %number of agents
Rrs = 1; %radius of repulsuion
Ros = 16;% %radius of orientation
Ras = inf; %radius of attraction
phis = 180/180*pi; %field of view
ks = .5; %maximum turn angle
ss = 5;  %Speed
nmag = 0; %Error in radians



%Simulation Params
DT = .1; %Time Step increment
T = 200;  %simulation time
trials = 100; %how many runs
plotsize = 100;%map size
ShowPlot = true;
TrackMoment = false;

for N = Ns
for Rr = Rrs
for Ro = Ros
for Ra = Ras
for k = ks
for s = ss
for trial = 1:trials
    count = 1;
    pgs = []; %group polarizations for tracking
    mgs = []; %group moments
    cgs = []; %group centers
    dgs = []; %group directions

    %initialize Agents
    p = [10*(2*rand(N,1)-1), 10*(2*rand(N,1)-1)];
    %p = [.1*(2*rand(N,1)-1), .1*(2*rand(N,1)-1)];
    theta = 2*pi*rand(N,1);
    d = [cos(theta), sin(theta)];
    dd = zeros(N,2);
    dtheta = zeros(N,1);
    
    if (task == FLOCK)
        d = repmat([cos(0) sin(0)],N,1);
    elseif task == TORUS
        circTheta = 0:2*pi/N:2*pi-2*pi/N;
        p = 4*s/k/pi*[cos(circTheta') sin(circTheta')];
        theta = atan2(p(:,2),p(:,1));
        d =  [cos(theta+pi/2) sin(theta+pi/2)];
    end
    
    %Run
    for t = 0:DT:T
        
        for i = 1:N
            di = d(i,:);
            pme = p(i,:);
            noti = [1:i-1 i+1:(N)];
            dnoti = d(noti,:);
            pnoti = p(noti,:);
            dvecs = pnoti - repmat(pme,length(noti),1);
            disti = sqrt(sum(dvecs.*dvecs,2));
            disti = max(disti,0.000001);

            %compute agent interactions    
            Ai = 1./[disti] > rand(N-1,1);%ones(N-1,1);%

            inRr = disti<=Rr;
            inRo = disti<=Ro;
            inRa = disti<=Ra;%(Ro <= disti) &(disti<=Ra);
            inRo = inRo & Ai;
            inRa = inRa & Ai;
            
%             inRrl = norm(C_d - pme)<Rrl;
%             inRol = (Rrl<norm(C_d - pme)) && (norm(C_d - pme)<Rol);
%             inRal = (Rol<norm(C_d-pme)) && (norm(C_d - pme)<Ral);
            
                
            %calculate desired direction
            
            repel = (-(dvecs./[disti,disti].^2)' * inRr)' ;
            orient = ((dnoti./[disti,disti].^0)' * inRo)';
            orient = (orient+di)/max(norm((orient+di)),.00001);
            attract = ((dvecs./[disti,disti].^0)' * inRa)';
            attract = attract/max(norm(attract),.00001);
       

            
            oa = (orient+attract+repel)/max(norm(orient+attract+repel),.00001);
            dd(i,:) =  oa;

        end
        
        
        %compute new headings and update position
        for i = 1:N
            angle = atan2(d(i,y),d(i,x));
            angled = atan2(dd(i,y),dd(i,x));
            dangle = angled - angle;
            if abs(dangle) > pi %make sure the chosen direction is the shortest
                if dangle >0
                    dangle = dangle - 2*pi;
                else
                    dangle = dangle + 2*pi;
                end
            end
            
           noise = nmag*(2*rand()-1)*DT;
           new_angle = dangle*DT*k + angle + noise;
           %new_angle = sign(dangle)*pi/8*DT + angle;
           

            %update heading
            d(i,:) = [cos(new_angle), sin(new_angle)];
            %update position
            p(i,:) = p(i,:) + s * DT * d(i,:);
           
        end
        

 
        pgroup = 1/N*norm(sum(d(1:N,:),1));
        cgroup = 1/N*sum(p(1:N,:));
       
        ri = p(1:N,:) - repmat(cgroup,N,1);
        ri = ri./repmat(sqrt(sum(ri.*ri,2)),1,2);

        mgroup = 1/N*abs(sum(ri(:,x).*d(1:N,y)) - sum(ri(:,y).*d(1:N,x)));
        dgroup = 1/N*sum(d(1:N,:),1);
      
        group_direction = atan2(dgroup(y),dgroup(x))*180/pi;
        
        %plot
        if ShowPlot
            if (t==0)
                figure(1); clf;
                hold on
                handle0= plot(p(:,x),p(:,y),'bo','EraseMode', 'Normal');
                handle3 = plot(cgroup(x),cgroup(y),'rx','EraseMode', 'Normal');
                
                h_dir_vectors = zeros(1,N+1);
                for i = 1:N
                    h_dir_vectors(i) =line([p(i,1),p(i,1)+1*d(i,1)],[p(i,2),p(i,2)+1*d(i,2)]);
                end
                
                
                axis([-plotsize plotsize -plotsize plotsize])
            else
                set(handle0, 'XData',p(:,x),'YData',p(:,y));
                set(handle3, 'XData',cgroup(x),'YData',cgroup(y));
                
                for i = 1:N
                    set(h_dir_vectors(i),'XData',[p(i,x),p(i,x)+2*d(i,x)], 'YData',[p(i,y),p(i,y)+2*d(i,y)]);
                end
                axis([-plotsize+cgroup(1) plotsize+cgroup(1) -plotsize+cgroup(2) plotsize+cgroup(2)])
                
                drawnow;
            end
        end
        

       
        
        if TrackMoment %store the group positions and leader positions at each time step
            mgs(count) = mgroup;
            pgs(count) = pgroup;
            cgs(count,:) = cgroup;
            count = count+1;
            
        end
        
    end %end  time
    
    if TrackMoment %save the group positions and leader positions
        filename = ['emergent=' num2str(Ro*100) ' trial=' num2str(trial)];
        save(filename)
    end
    
end
end
end
end
end
end
end