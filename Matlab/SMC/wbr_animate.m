function wbr_animate(T, Qlog, TAUlog, p, opts)
%WBR_ANIMATE  Animazione 2D del bipede su ruote.
%
%   wbr_animate(T, Qlog, TAUlog, p, opts)
%     T      : Nx1  vettore tempi [s]
%     Qlog   : Nx4  [q1 qw q2 q3] (rad)
%     TAUlog : Nx3  [tau_w tau_2 tau_3] (Nm)   -> [] per ometterle
%     p      : struct parametri (da robot_params.mat)
%     opts   : struct opzionale
%                .speed    fattore di rallentamento (default 1)
%                .fps      frame rate (default 40)
%                .record   'none' | 'gif' | 'mp4'   (default 'none')
%                .file     nome file senza estensione (default 'wbr')
%                .follow   true = camera che insegue il robot (default true)
%
%   Convenzioni geometriche (coerenti col modello):
%     - contatto ruota-suolo:   (Rw*qw, 0)
%     - centro ruota:           (Rw*qw, Rw)   <- origine della catena
%     - stinco:  angolo ASSOLUTO q1 dall'orizzontale
%     - coscia:  angolo ASSOLUTO q1+q2
%     - torso:   angolo ASSOLUTO q3 dalla VERTICALE  (x ~ sin q3, y ~ cos q3)

if nargin < 5, opts = struct; end
if ~isfield(opts,'speed'),  opts.speed  = 1;      end
if ~isfield(opts,'fps'),    opts.fps    = 40;     end
if ~isfield(opts,'record'), opts.record = 'none'; end
if ~isfield(opts,'file'),   opts.file   = 'wbr';  end
if ~isfield(opts,'follow'), opts.follow = true;   end

Rw=p.Rw; l1=p.l1; l2=p.l2; l3=p.l3;
m1=p.m1; m2=p.m2; m3=p.m3; mw=p.mw; mb=m1+m2+m3; mt=mb+mw;

%---- decimazione a fps richiesto ---------------------------------------
dt   = mean(diff(T));
step = max(1, round(1/(opts.fps*dt)));
idx  = 1:step:numel(T);

%---- pre-calcolo cinematica -------------------------------------------
N = numel(idx);
P = zeros(N,9);   % [sw, xk,yk, xh,yh, xt,yt, xc,yc]  (+1 slack)
TH = zeros(N,1); VG = zeros(N,1); XG = zeros(N,1);
for i = 1:N
    q = Qlog(idx(i),:);
    sw = Rw*q(2);
    x0 = sw;            y0 = Rw;                                   % centro ruota
    xk = x0 + l1*cos(q(1));            yk = y0 + l1*sin(q(1));     % ginocchio
    xh = xk + l2*cos(q(1)+q(3));       yh = yk + l2*sin(q(1)+q(3));% anca
    xt = xh + l3*sin(q(4));            yt = yh + l3*cos(q(4));     % testa torso

    % baricentri di link
    c1 = [x0 + l1/2*cos(q(1)),            y0 + l1/2*sin(q(1))];
    c2 = [xk + l2/2*cos(q(1)+q(3)),       yk + l2/2*sin(q(1)+q(3))];
    c3 = [xh + l3/2*sin(q(4)),            yh + l3/2*cos(q(4))];
    cu = (m1*c1 + m2*c2 + m3*c3)/mb;                                % CoM upper body
    P(i,:) = [sw, xk,yk, xh,yh, xt,yt, cu(1),cu(2)];
    TH(i)  = atan2(cu(1)-sw, cu(2));                                % theta_b
    XG(i)  = (mw*sw + m1*c1(1) + m2*c2(1) + m3*c3(1))/mt;           % CoM totale (x)
end
VG(2:end) = diff(XG)./diff(T(idx));  VG(1) = VG(2);

%---- figura ------------------------------------------------------------
fig = figure('Name','Bipede su ruote - SMC','Color','w','Position',[80 80 1180 620]);
axR = subplot(1,3,[1 2]); hold(axR,'on'); axis(axR,'equal'); grid(axR,'on');
xlabel(axR,'x [m]'); ylabel(axR,'y [m]');

% suolo
xmin = min(P(:,1))-1.0; xmax = max(P(:,1))+1.0;
plot(axR,[xmin xmax],[0 0],'k','LineWidth',2);
for xs = xmin:0.08:xmax
    plot(axR,[xs xs-0.05],[0 -0.05],'Color',[.6 .6 .6],'LineWidth',0.5);
end
ylim(axR,[-0.10 1.55]);

% handle grafici
hWheel = plot(axR,0,0,'k','LineWidth',2.0);
hSpoke = plot(axR,0,0,'k','LineWidth',1.0);
hPend  = plot(axR,0,0,'--','Color',[0.85 0.33 0.10],'LineWidth',1.2);   % pendolo equiv.
hLeg   = plot(axR,0,0,'-','Color',[0 0.45 0.74],'LineWidth',4.0);        % stinco+coscia
hTorso = plot(axR,0,0,'-','Color',[0.15 0.15 0.15],'LineWidth',7.0);
hJoint = plot(axR,0,0,'o','MarkerSize',7,'MarkerFaceColor','w','Color','k');
hCoMu  = plot(axR,0,0,'o','MarkerSize',10,'MarkerFaceColor',[0.85 0.33 0.10], ...
              'MarkerEdgeColor','k');
hCoMt  = plot(axR,0,0,'p','MarkerSize',13,'MarkerFaceColor',[0.47 0.67 0.19], ...
              'MarkerEdgeColor','k');
hTrail = plot(axR,NaN,NaN,':','Color',[0.85 0.33 0.10],'LineWidth',1.0);
hTxt   = text(axR,0,0,'','FontName','Consolas','FontSize',10, ...
              'BackgroundColor',[1 1 1 ],'EdgeColor',[.7 .7 .7],'Margin',4);
legend(axR,[hCoMu hCoMt hPend],{'CoM upper body','CoM totale','pendolo equivalente'}, ...
       'Location','northwest'); 

% telemetria
axT1 = subplot(3,3,3); hold(axT1,'on'); grid(axT1,'on');
plot(axT1,T(idx),rad2deg(TH),'Color',[0.85 0.33 0.10]);
hM1 = plot(axT1,T(1),rad2deg(TH(1)),'ko','MarkerFaceColor','k','MarkerSize',4);
ylabel(axT1,'\theta_b [deg]');

axT2 = subplot(3,3,6); hold(axT2,'on'); grid(axT2,'on');
plot(axT2,T(idx),VG,'Color',[0 0.45 0.74]);
hM2 = plot(axT2,T(1),VG(1),'ko','MarkerFaceColor','k','MarkerSize',4);
ylabel(axT2,'v_{com} [m/s]');

axT3 = subplot(3,3,9); hold(axT3,'on'); grid(axT3,'on');
if ~isempty(TAUlog)
    plot(axT3,T(idx),TAUlog(idx,:));
    legend(axT3,{'\tau_w','\tau_2','\tau_3'},'Location','best','FontSize',7);
end
hM3 = plot(axT3,T(1),0,'ko','MarkerFaceColor','k','MarkerSize',4);
ylabel(axT3,'\tau [Nm]'); xlabel(axT3,'t [s]');

%---- registrazione ------------------------------------------------------
switch lower(opts.record)
    case 'mp4', vw = VideoWriter([opts.file '.mp4'],'MPEG-4');
                vw.FrameRate = opts.fps; open(vw);
    case 'gif', vw = [];
    otherwise,  vw = [];
end

th_c = linspace(0,2*pi,60);
tic;
for i = 1:N
    sw = P(i,1); xc = P(i,8); yc = P(i,9);
    q  = Qlog(idx(i),:);

    % ruota: rotolamento puro -> l'angolo dei raggi e' -qw
    set(hWheel,'XData',sw + Rw*cos(th_c),'YData',Rw + Rw*sin(th_c));
    sa = -q(2) + [0 pi/2 pi 3*pi/2];
    sx = reshape([sw+zeros(1,4); sw+Rw*cos(sa); nan(1,4)],1,[]);
    sy = reshape([Rw+zeros(1,4); Rw+Rw*sin(sa); nan(1,4)],1,[]);
    set(hSpoke,'XData',sx,'YData',sy);

    set(hLeg,  'XData',[sw P(i,2) P(i,4)],'YData',[Rw P(i,3) P(i,5)]);
    set(hTorso,'XData',[P(i,4) P(i,6)],   'YData',[P(i,5) P(i,7)]);
    set(hJoint,'XData',[sw P(i,2) P(i,4)],'YData',[Rw P(i,3) P(i,5)]);
    set(hPend, 'XData',[sw xc],'YData',[0 yc]);          % dal CONTATTO al CoM
    set(hCoMu, 'XData',xc,'YData',yc);
    set(hCoMt, 'XData',XG(i),'YData',0.02);              % proiezione a terra
    set(hTrail,'XData',P(1:i,8),'YData',P(1:i,9));

    set(hTxt,'Position',[sw-0.55, 1.42], 'String', sprintf( ...
        't = %5.2f s\n\\theta_b = %+6.2f deg\nv_{com} = %+5.3f m/s\nq_2 = %+6.1f deg', ...
        T(idx(i)), rad2deg(TH(i)), VG(i), rad2deg(q(3))));

    set(hM1,'XData',T(idx(i)),'YData',rad2deg(TH(i)));
    set(hM2,'XData',T(idx(i)),'YData',VG(i));
    if ~isempty(TAUlog), set(hM3,'XData',T(idx(i)),'YData',TAUlog(idx(i),1)); end

    if opts.follow, xlim(axR,[sw-1.0, sw+1.0]); else, xlim(axR,[xmin xmax]); end

    drawnow limitrate;

    switch lower(opts.record)
        case 'mp4', writeVideo(vw, getframe(fig));
        case 'gif'
            fr = getframe(fig); [A,map] = rgb2ind(frame2im(fr),256);
            if i==1, imwrite(A,map,[opts.file '.gif'],'gif','LoopCount',Inf, ...
                             'DelayTime',1/opts.fps);
            else,    imwrite(A,map,[opts.file '.gif'],'gif','WriteMode','append', ...
                             'DelayTime',1/opts.fps);
            end
        otherwise
            % tempo reale (rallentato di opts.speed)
            while toc < i*step*dt*opts.speed, end
    end
end
if strcmpi(opts.record,'mp4'), close(vw); end
end
