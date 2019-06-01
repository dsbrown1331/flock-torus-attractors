%In this model we have type-aware agents that have an awareness parameter
%that causes them to ignore or be aware of human-aware agetnts to hopefully
%create a vulnerability threshold with Q. If agents don't sense a quorum of
%stakeholders they ignore the stakeholders as noisy failed/stupid agents,
%but once they sense a quorum they temporariliy increase their awareness
%parameter. awareness = 0 => only respond to nonstakeholders. awareness = 1
%=> only respond to stakeholders.

clear all
close all

x=1;
y=2;

%tasks
FLOCK2TORUS = 0;
TORUS2FLOCK = 1;

%leadership styles
ATTRACTION = 0;
ORIENTATION = 1;

leadstyle = ORIENTATION;

task = TORUS2FLOCK;

tic

%agent parameters
Ns = 100; %number of agents
Rrs = 1; %radius of repulsuion
Ros = 8;% %radius of orientation
Ras = inf; %radius of attraction
phis = 180/180*pi; %field of view
ks = .5; %maximum turn angle
ss = 5; % %Speed
nmag = 0; %Error in radians

%%%this stuff if for the quorum sensing work
%type aware agents
W = 0; %this gets set inside the loops
Q = 0; %quorum threshold
minAwareness = 0;  %no quorum
maxAwareness = 1;  %quorum
%%%

%control parameters for stakeholders
Ms = 40 %number of stakeholders[10 20 30 40 50 60  70 80 90 100];
sls = 5;  %speed of stakeholders
Cdi = [0,0]; %position of control waypoint
Vd = [0,0]; %reference heading velocity
Cmags = 0.8; %stakeholder priority [.1 .2 .3 .4 .5 .6 .7 .8 .9 1];


%memory time
%Communication frequency

%Simulation Params
DT = .1; %Time Step increment
T = 150; %simulation time
trials = 1; %how many runs
plotsize = 100;%map size
WriteData = false;
ShowPlot = true
TrackMoment = false;

StartTime = 25;  %when to start stakeholder influence
StopTime = 125;  %when to end stakeholder influence


if WriteData
    fout = fopen('outdata.txt','w');
end

for N = Ns
for M = Ms
    M
    %W = N - M
for Rr = Rrs
for Ro = Ros
for Ra = Ras
for phi = phis
for rho = Cmags
    rho
for k = ks
for s = ss
for sl = sls
    b =[ones(M,1);zeros(N-M,1)];
    aware = [zeros(N-W,1);ones(W,1)];
for trial = 1:trials
    trial
    count = 1;

    ticks = length(0:DT:T);
    headings = cell(ticks,1);
    position = cell(ticks,1);
    Neighbors = cell(ticks,1);
    angVel = cell(ticks,1);
    pgs = zeros(ticks,1); %group polarizations for tracking
    mgs = zeros(ticks,1); %group moments
    cgs = zeros(ticks,2); %group centers
    dgs = zeros(ticks,1); %group directions
    

    Cd = Cdi;
    integrator = 0;
    thetal = pi/2;
    %initialize Agents
    p = [10*(2*rand(N,1)-1), 10*(2*rand(N,1)-1)];
    theta = 2*pi*rand(N,1);
    d = [cos(theta), sin(theta)];
    dd = zeros(N,2);
    dtheta = zeros(N,1);
    Cmag = 0; %start off as zero and then switch on and off
    



    
    if task == FLOCK2TORUS
        d = repmat([cos(0) sin(0)],N,1);
    elseif task == TORUS2FLOCK
        theta = atan2(p(:,2),p(:,1));
        d =  [cos(theta+pi/2) sin(theta+pi/2)];
    end
    
    sensedStakeholders = zeros(N,1);
    %Run
    for t = 0:DT:T
        
        A = zeros(N,N);
       
        if(t==StartTime)
            if task == TORUS2FLOCK
                Cmag = rho;
                Cd = cgroup + [10000,0];
            elseif task == FLOCK2TORUS
                Cmag = rho;
                Cd = cgroup + [0,10];
            end
        end
        
        if(t==StopTime)
            Cmag = 0;
        end
        
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
            A(i,:) = [Ai(1:i-1); 0; Ai(i:end)]';
            
            %compute how many stakeholders are neighbors
            if Cmag == 0
                sensedStakeholders(i) = 0;
            else
                sensedStakeholders(i) = [ones(1,M) zeros(1,N-M)]*[Ai(1:i-1); 0; Ai(i:end)];
            end

            inRr = disti<=Rr;
            inRo = disti<=Ro;
            inRa = disti<=Ra;
            inRo = inRo & Ai;
            inRa = inRa & Ai;
            
           
            
                
            %calculate desired direction
            
            repel = (-(dvecs./[disti,disti].^2)' * inRr)' ;
            
            
                      
            %check if agent i is type aware and has a quorum number of
            %stakeholder neighbors
            if aware(i) && sensedStakeholders(i)>0
                if(sensedStakeholders(i) > Q)
                    awareness = maxAwareness;
                else
                    awareness = minAwareness;
                end
                 %compute the orientation, and attraction zones
                 %rules when responding to only my stakeholder neighbors
                 stakeholders = b;
                 stakeholders(i) = [];
                 inRoStake = inRo.*stakeholders;
                 inRaStake = inRa.*stakeholders;
                 inRoBlind = inRo.*~stakeholders;
                 inRaBlind = inRa.*~stakeholders;
                       
                 awareOrient = ((dnoti./[disti,disti].^0)' * inRoStake)';
                 awareOrient = (awareOrient+di)/max(norm((awareOrient+di)),.00001);
                 awareAttract = ((dvecs./[disti,disti].^0)' * inRaStake)';
                 awareAttract = awareAttract/max(norm(awareAttract),.00001);
                 
                 blindOrient = ((dnoti./[disti,disti].^0)' * inRoBlind)';
                 blindOrient = (blindOrient+di)/max(norm((blindOrient+di)),.00001);
                 blindAttract = ((dvecs./[disti,disti].^0)' * inRaBlind)';
                 blindAttract = blindAttract/max(norm(blindAttract),.00001);
                 
                 
                 orient = (awareness*awareOrient+(1-awareness)*blindOrient)/max(norm(awareness*awareOrient+(1-awareness)*blindOrient),.00001);
                 attract = (awareness*awareAttract+(1-awareness)*blindAttract)/max(norm(awareness*awareAttract+(1-awareness)*blindAttract),.00001);
            else
                orient = ((dnoti./[disti,disti].^0)' * inRo)';
                orient = (orient+di)/max(norm((orient+di)),.00001);
                attract = ((dvecs./[disti,disti].^0)' * inRa)';
                attract = attract/max(norm(attract),.00001);
                stake =   (Cd-pme)/max(norm(Cd-pme),.00001);
                if leadstyle == ORIENTATION
                    orient = (orient*(~b(i))+b(i)*(Cmag*stake+(1-Cmag)*orient))/max(norm((orient*(~b(i))+b(i)*(Cmag*stake+(1-Cmag)*orient))),.00001);
                elseif leadstyle == ATTRACTION
                    attract = (attract*(~b(i))+b(i)*(Cmag*stake+(1-Cmag)*attract))/max(norm((attract*(~b(i))+b(i)*(Cmag*stake+(1-Cmag)*attract))),.00001);
                end
            end

            oa = (orient+attract+repel)/max(norm(orient+attract+repel),.00001);
            dd(i,:) =  oa;


            %Heading vector, position vector, velocity stabilization.


            
            if isnan(dd(i,x))
                err = MException('ResultChk:OutOfRange', ...
                    'Resulting value is outside expected range');
                throw(err)
            end
        end
        
        
        %compute new headings and update position
       
        angle = atan2(d(:,y),d(:,x));
        angled = atan2(dd(:,y),dd(:,x));
        dangle = angled - angle;
        dangle = dangle - 2*pi*(dangle > pi) + 2*pi*(dangle < -pi);
        

        ang_vel = k*dangle;
        noise = nmag*(2*rand(N,1)-1)*DT;
        new_angle = angle + ang_vel*DT + noise;
        

        %update heading
        d = [cos(new_angle), sin(new_angle)];
        %update position
        p = p + s * DT * d;
        

 
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
                handle0= plot(p(1:M:end,x),p(1:M:end,y),'ro','EraseMode', 'Normal');
                handle1= plot(p((M+1):end-W,x),p((M+1):end-W,y),'bo','EraseMode', 'Normal');
                handle2 = plot(Cd(x),Cd(y),'rO','EraseMode', 'Normal');
                handle3 = plot(cgroup(x),cgroup(y),'rx','EraseMode', 'Normal');
                handleTypeAware = zeros(1,W);
                for i = 1:W
                   handleTypeAware(i) = plot(p((end-W+1):end,x),p((end-W+1):end,y),'g^');
                end
                
                h_dir_vectors = zeros(1,N+1);
                for i = 1:N
                    h_dir_vectors(i) =line([p(i,1),p(i,1)+1*d(i,1)],[p(i,2),p(i,2)+1*d(i,2)]);
                end
                
                
                axis([-plotsize plotsize -plotsize plotsize])
            else
                if(Cmag == 0)
                    set(handle0, 'XData',p(1:M,x),'YData',p(1:M,y),'markerface','w');
                else
                    set(handle0, 'XData',p(1:M,x),'YData',p(1:M,y),'markerface','r');
                end
                set(handle1, 'XData',p((M+1):end-W,x),'YData',p((M+1):end-W,y));
                set(handle2, 'XData',Cd(x),'YData',Cd(y));
                set(handle3, 'XData',cgroup(x),'YData',cgroup(y));
    
                for i = 1:W
                    if(sensedStakeholders(N-W+i) > Q)
                        set(handleTypeAware(i),'Xdata',p(N-W+i,x),'YData',p(N-W+i,y),'markerfacecolor','g');
                    else
                        set(handleTypeAware(i),'Xdata',p(N-W+i,x),'YData',p(N-W+i,y),'markerfacecolor','w');
                    end
                end
                
                for i = 1:N
                    set(h_dir_vectors(i),'XData',[p(i,x),p(i,x)+2*d(i,x)], 'YData',[p(i,y),p(i,y)+2*d(i,y)]);
                end
                axis([-plotsize+cgroup(1) plotsize+cgroup(1) -plotsize+cgroup(2) plotsize+cgroup(2)])
                title(t);
                drawnow;
            end
        end
        

       
        
        if TrackMoment %store the group positions and leader positions at each time step
            %headings{count} = new_angle;
            %position{count} = p;
            %angVel{count} = ang_vel;
            Neighbors{count} = A;
            mgs(count) = mgroup;
            pgs(count) = pgroup;
            %cgs(count,:) = cgroup;
            %cds(count,:) = Cd;
            %dcs(count) = norm((Cd)-cgroup);
            count = count+1;
        end
        
        
        Cd = Cd + sl*DT*Vd;

        


        
    end %end  time
    
    if TrackMoment %save the group positions and leader positions
        filename = ['t2fo=' num2str(trial) 'M=' num2str(M) 'rho=' num2str(rho*100) 'Ro=' num2str(Ro)];
        save(filename)
    end
    
end
end
end
end
end
end
end
end
end
end
end



toc