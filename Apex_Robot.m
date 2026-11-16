function Apex_Robot(varargin)

    cfg = parseOptions(defaultConfig(), varargin);

    fprintf('Apex_Robot: running start-up self-tests...\n');
    runSelfTests(cfg);                 
    fprintf('Apex_Robot: self-tests passed.\n');

    S = buildScene(cfg);
    try
        S = runCommandLoop(S); 
        fprintf('Apex_Robot command session ended.\n');
    catch ME
        if strcmp(ME.identifier, 'RoboMind:figureClosed')
            fprintf('Apex_Robot: window closed - command session stopped.\n');
        else
            rethrow(ME);
        end
    end
end


function cfg = defaultConfig()
   
    cfg.L1 = 0.60;                 
    cfg.L2 = 0.55;                 
    cfg.Lg = 0.22;                 
    cfg.z0 = 0.45;                 
    cfg.qMin = [-Inf -0.35 -2.70 -2.80];   
    cfg.qMax = [ Inf  2.40 -0.05  2.80];
    cfg.minReach = 0.30;           
    cfg.homeTip  = [0.50 0 0.60];  

    
    cfg.safeZ      = 0.48;         
    cfg.detectDist = 0.55;         
    cfg.gapOpen    = 0.11;         
    cfg.gapIdle    = 0.03;         
    cfg.fingerHalfT = 0.01;        
    cfg.minGraspZ  = 0.04;         
    cfg.verifyTol  = 0.010;        

    
    cfg.stationTopZ = 0.02;
    cfg.stationY    = 0.72;
    cfg.slotX       = -0.64:0.32:0.64;   

   
    cfg.perceptionBackend = 'simulated';
    cfg.classifierBackend = 'simulated';

    
    cfg.speed   = 1;
    cfg.visible = 'on';
    cfg.finalMsg = ['ALL OBJECTS SORTED SUCCESSFULLY ' char(8211) ' ROBOT RETURNED HOME.'];

   
    cfg.objects = struct( ...
        'label', {'Cube', 'Sphere', 'Cylinder', 'Triangular prism', 'Circular disk'}, ...
        'color', {[0.90 0.15 0.15], [0.15 0.75 0.30], [1.00 0.55 0.10], ...
                  [0.55 0.25 0.85], [0.15 0.45 0.95]}, ...
        'pos',   {[0.75 -0.30], [0.45 -0.65], [0.95 0.10], [0.30 -0.40], [-0.55 -0.45]});
end

function cfg = parseOptions(cfg, args)
    if mod(numel(args), 2) ~= 0
        error('RoboMind:options', 'Options must be name/value pairs.');
    end
    for i = 1:2:numel(args)
        name = lower(char(args{i}));
        val  = args{i+1};
        switch name
            case 'speed'
                if ~isnumeric(val) || ~isscalar(val) || val <= 0
                    error('RoboMind:options', 'Speed must be a positive scalar.');
                end
                cfg.speed = double(val);
            case 'visible'
                cfg.visible = char(val);
            otherwise
                error('RoboMind:options', 'Unknown option "%s".', name);
        end
    end
end


function runSelfTests(cfg)
   
    tests = { meshBox([0 0 0], [1 2 3]),          'box',        [1 2 3], 6;
              meshCylinder([0 0 0], 0.5, 2, 'x'), 'cylinder-x', [2 1 1], 40/2*0.25*sin(2*pi/40)*2;
              meshCylinder([0 0 0], 0.5, 2, 'y'), 'cylinder-y', [1 2 1], 40/2*0.25*sin(2*pi/40)*2;
              meshCylinder([0 0 0], 0.5, 2, 'z'), 'cylinder-z', [1 1 2], 40/2*0.25*sin(2*pi/40)*2 };
    for i = 1:size(tests, 1)
        M = tests{i, 1};
        validateMesh(M, tests{i, 2});
        ext = max(M.V, [], 1) - min(M.V, [], 1);
        if any(abs(ext - tests{i, 3}) > 1e-9)
            error('RoboMind:selfTest', 'Mesh "%s" has wrong extents.', tests{i, 2});
        end
        if abs(meshVolume(M) - tests{i, 4}) > 1e-9
            error('RoboMind:selfTest', 'Mesh "%s" has wrong volume / orientation.', tests{i, 2});
        end
    end
    Mp = meshPrism([0 0 0], 1, 1);
    validateMesh(Mp, 'prism');
    if abs(meshVolume(Mp) - 3*sqrt(3)/4) > 1e-9
        error('RoboMind:selfTest', 'Prism mesh has wrong volume / orientation.');
    end
    Ms = meshSphere([0 0 0], 1, 16);
    validateMesh(Ms, 'sphere');
    if meshVolume(Ms) < 0.9*4*pi/3 || meshVolume(Ms) > 4*pi/3
        error('RoboMind:selfTest', 'Sphere mesh has wrong volume / orientation.');
    end
    for k = 1:numel(cfg.objects)
        g = objectSpec(cfg.objects(k).label);
        validateMesh(g.mesh, cfg.objects(k).label);
        if meshVolume(g.mesh) <= 0
            error('RoboMind:selfTest', 'Object mesh "%s" is inside-out.', cfg.objects(k).label);
        end
        [P, ~] = slotOutline(cfg.objects(k).label);
        if size(P, 2) ~= 2 || size(P, 1) < 3
            error('RoboMind:selfTest', 'Bad slot outline for "%s".', cfg.objects(k).label);
        end
    end

    
    [~, ok] = ikSolve(cfg.homeTip, cfg);
    if ~ok, error('RoboMind:selfTest', 'Home position is unreachable.'); end
    nOk = 0;
    for r = 0.45:0.1:1.0
        for z = 0.04:0.12:0.52
            for th = -pi:pi/4:pi
                p = [r*cos(th), r*sin(th), z];
                [q, ok] = ikSolve(p, cfg);
                if ok
                    T = fkMatrix(q, cfg);
                    if norm(T(1:3, 4)' - p) > 1e-9
                        error('RoboMind:selfTest', 'FK(IK(p)) mismatch at [%.2f %.2f %.2f].', p);
                    end
                    if abs(T(3, 1) + 1) > 1e-9
                        error('RoboMind:selfTest', 'Tool is not vertical at [%.2f %.2f %.2f].', p);
                    end
                    nOk = nOk + 1;
                end
            end
        end
    end
    if nOk == 0, error('RoboMind:selfTest', 'IK solved no sample points.'); end

    
    qt = [0.4 0.7 -1.3 -0.97];
    F  = fkFrames(qt, cfg);
    Tc = F.yaw * F.shoulder * F.elbow * F.wrist * Tr4([cfg.Lg 0 0]);
    if norm(Tc - fkMatrix(qt, cfg), 'fro') > 1e-12
        error('RoboMind:selfTest', 'Frame chain differs from fkMatrix.');
    end

    
    nObj = numel(cfg.objects);
    if numel(cfg.slotX) ~= nObj
        error('RoboMind:selfTest', 'Need exactly one slot x-position per object.');
    end
    P = vertcat(cfg.objects.pos);
    for a = 1:nObj
        for b = a+1:nObj
            if norm(P(a, :) - P(b, :)) < 0.20
                error('RoboMind:selfTest', 'Objects %d and %d are too close together.', a, b);
            end
        end
    end
    for k = 1:nObj
        o  = cfg.objects(k);
        g  = objectSpec(o.label);
        gz = max(g.halfH, cfg.minGraspZ);
        sx = cfg.slotX(k);  sy = cfg.stationY;
        pts = [o.pos(1) o.pos(2) gz;
               o.pos(1) o.pos(2) cfg.safeZ;
               sx       sy       cfg.safeZ;
               sx       sy       cfg.stationTopZ + gz];
        for m = 1:size(pts, 1)
            [~, ok] = ikSolve(pts(m, :), cfg);
            if ~ok
                error('RoboMind:selfTest', 'Unreachable target for %s: [%.2f %.2f %.2f].', ...
                      o.label, pts(m, :));
            end
        end
    end
end

function validateMesh(M, name)
    if ~isstruct(M) || ~isfield(M, 'V') || ~isfield(M, 'F')
        error('RoboMind:mesh', 'Mesh "%s" is not a V/F struct.', name);
    end
    if size(M.V, 2) ~= 3 || size(M.F, 2) ~= 3
        error('RoboMind:mesh', 'Mesh "%s": V must be Nx3 and F must be Mx3.', name);
    end
    if any(~isfinite(M.V(:)))
        error('RoboMind:mesh', 'Mesh "%s" has non-finite vertices.', name);
    end
    if any(M.F(:) < 1) || any(M.F(:) > size(M.V, 1)) || any(M.F(:) ~= round(M.F(:)))
        error('RoboMind:mesh', 'Mesh "%s" has invalid face indices.', name);
    end
end

function v = meshVolume(M)
    
    a = M.V(M.F(:, 1), :);
    b = M.V(M.F(:, 2), :);
    c = M.V(M.F(:, 3), :);
    v = sum(sum(a .* cross(b, c, 2), 2)) / 6;
end


function T = Rz4(a)
    T = [cos(a) -sin(a) 0 0; sin(a) cos(a) 0 0; 0 0 1 0; 0 0 0 1];
end

function T = Ry4(a)
    T = [cos(a) 0 sin(a) 0; 0 1 0 0; -sin(a) 0 cos(a) 0; 0 0 0 1];
end

function T = Tr4(v)
    T = eye(4);
    T(1:3, 4) = v(:);
end

function T = fkMatrix(q, cfg)
    T = Rz4(q(1)) * Tr4([0 0 cfg.z0]) * Ry4(-q(2)) * Tr4([cfg.L1 0 0]) ...
        * Ry4(-q(3)) * Tr4([cfg.L2 0 0]) * Ry4(-q(4)) * Tr4([cfg.Lg 0 0]);
end

function p = fkTip(q, cfg)
    T = fkMatrix(q, cfg);
    p = T(1:3, 4)';
end

function F = fkFrames(q, cfg)
    
    F.yaw      = Rz4(q(1));
    F.shoulder = Tr4([0 0 cfg.z0]) * Ry4(-q(2));
    F.elbow    = Tr4([cfg.L1 0 0]) * Ry4(-q(3));
    F.wrist    = Tr4([cfg.L2 0 0]) * Ry4(-q(4));
end

function [q, ok] = ikSolve(p, cfg)
    
    q  = nan(1, 4);
    ok = false;
    r  = hypot(p(1), p(2));
    if r < cfg.minReach, return; end
    q1 = atan2(p(2), p(1));
    dx = r;                              
    dz = p(3) + cfg.Lg - cfg.z0;
    D  = hypot(dx, dz);
    if D > cfg.L1 + cfg.L2 - 1e-6 || D < abs(cfg.L1 - cfg.L2) + 1e-6, return; end
    c3 = (D^2 - cfg.L1^2 - cfg.L2^2) / (2 * cfg.L1 * cfg.L2);
    c3 = max(-1, min(1, c3));
    q3 = -acos(c3);                      
    q2 = atan2(dz, dx) - atan2(cfg.L2 * sin(q3), cfg.L1 + cfg.L2 * cos(q3));
    q4 = -pi/2 - q2 - q3;
    qc = [q1 q2 q3 q4];
    if any(qc < cfg.qMin) || any(qc > cfg.qMax), return; end
    q  = qc;
    ok = true;
end

function q = alignYaw(q, qRef)
    
    q(1) = qRef(1) + mod(q(1) - qRef(1) + pi, 2*pi) - pi;
end


function M = meshBox(c, s)
    h = s(:)' / 2;
    U = [-1 -1 -1;  1 -1 -1;  1 1 -1; -1 1 -1; ...
         -1 -1  1;  1 -1  1;  1 1  1; -1 1  1];
    V = U .* h + c(:)';
    F = [1 4 3; 1 3 2;     % bottom
         5 6 7; 5 7 8;     % top
         1 2 6; 1 6 5;     % -y
         4 8 7; 4 7 3;     % +y
         1 5 8; 1 8 4;     % -x
         2 3 7; 2 7 6];    % +x
    M = struct('V', V, 'F', F, 'smooth', false);
end

function M = meshCylinder(c, r, h, axisName)
    n  = 40;
    th = 2*pi*(0:n-1)' / n;
    x  = r * cos(th);
    y  = r * sin(th);
    V  = [x y -h/2*ones(n, 1); x y h/2*ones(n, 1); 0 0 -h/2; 0 0 h/2];
    F  = zeros(4*n, 3);
    for i = 1:n
        j = mod(i, n) + 1;
        F(4*i-3, :) = [i j n+j];            
        F(4*i-2, :) = [i n+j n+i];          
        F(4*i-1, :) = [2*n+1 j i];          
        F(4*i,   :) = [2*n+2 n+i n+j];     
    end
    switch axisName
        case 'x', V = V(:, [3 1 2]);        
        case 'y', V = V(:, [2 3 1]);
        case 'z'  
        otherwise, error('RoboMind:mesh', 'Cylinder axis must be x, y or z.');
    end
    V = V + c(:)';
    M = struct('V', V, 'F', F, 'smooth', false);
end

function M = meshSphere(c, r, n)
    [X, Y, Z] = sphere(n);                  
    V   = r * [X(:) Y(:) Z(:)] + c(:)';
    idx = reshape(1:(n+1)^2, n+1, n+1);
    F   = zeros(2*n*n, 3);
    m   = 0;
    for i = 1:n
        for j = 1:n
            a  = idx(i, j);   b = idx(i+1, j);
            cc = idx(i+1, j+1); d = idx(i, j+1);
            m = m + 1;  F(m, :) = [a cc b];
            m = m + 1;  F(m, :) = [a d cc];
        end
    end
    M = struct('V', V, 'F', F, 'smooth', true);
end

function M = meshPrism(c, R, h)
   
    a  = pi/2 + 2*pi*(0:2)' / 3;
    xy = R * [cos(a) sin(a)];
    V  = [xy -h/2*ones(3, 1); xy h/2*ones(3, 1)] + c(:)';
    F  = [1 3 2; 4 5 6];
    for i = 1:3
        j = mod(i, 3) + 1;
        F = [F; i j 3+j; i 3+j 3+i]; 
    end
    M = struct('V', V, 'F', F, 'smooth', false);
end


function g = objectSpec(label)
    switch label
        case 'Cube'
            mesh = meshBox([0 0 0], [0.12 0.12 0.12]);
            halfH = 0.06;  gripHalf = 0.06;  footR = 0.085;
        case 'Sphere'
            mesh = meshSphere([0 0 0], 0.07, 24);
            halfH = 0.07;  gripHalf = 0.07;  footR = 0.07;
        case 'Cylinder'
            mesh = meshCylinder([0 0 0], 0.05, 0.14, 'z');
            halfH = 0.07;  gripHalf = 0.05;  footR = 0.05;
        case 'Triangular prism'
            mesh = meshPrism([0 0 0], 0.09, 0.12);
            halfH = 0.06;  gripHalf = 0.075; footR = 0.09;
        case 'Circular disk'
            mesh = meshCylinder([0 0 0], 0.08, 0.03, 'z');
            halfH = 0.015; gripHalf = 0.08;  footR = 0.08;
        otherwise
            error('RoboMind:unknownShape', 'Unknown shape label "%s".', label);
    end
    g = struct('mesh', mesh, 'halfH', halfH, 'gripHalf', gripHalf, 'footR', footR);
end

function [P, typeName] = slotOutline(label)
    
    switch label
        case 'Cube'
            P = 0.10 * [-1 -1; 1 -1; 1 1; -1 1];
            typeName = 'SQUARE SLOT';
        case 'Triangular prism'
            a = pi/2 + 2*pi*(0:2)' / 3;               
            P = 0.13 * [cos(a) sin(a)];
            typeName = 'TRIANGULAR SLOT';
        case {'Sphere', 'Cylinder', 'Circular disk'}
            t = 2*pi*(0:47)' / 48;
            P = 0.115 * [cos(t) sin(t)];
            typeName = 'CIRCULAR SLOT';
        otherwise
            error('RoboMind:unknownShape', 'No slot defined for "%s".', label);
    end
end

function m = matProps(name)
    
    switch name
        case 'metal',   m = [0.40 0.65 0.95 45];
        case 'plastic', m = [0.45 0.80 0.50 25];
        case 'matte',   m = [0.50 0.75 0.10  8];
        otherwise,      m = [0.50 0.70 0.30 15];
    end
end

function h = addMesh(parent, M, color, matName)
    m = matProps(matName);
    if M.smooth, fl = 'gouraud'; else, fl = 'flat'; end
    h = patch('Parent', parent, 'Faces', M.F, 'Vertices', M.V, ...
              'FaceColor', color, 'EdgeColor', 'none', 'FaceLighting', fl, ...
              'AmbientStrength', m(1), 'DiffuseStrength', m(2), ...
              'SpecularStrength', m(3), 'SpecularExponent', m(4));
end


function S = buildScene(cfg)
    bg = [0.05 0.07 0.11];
    fig = figure('Name', 'Apex_Robot - 3D Robotic Sorting Simulation', ...
                 'NumberTitle', 'off', 'Color', bg, 'Units', 'normalized', ...
                 'OuterPosition', [0.03 0.05 0.94 0.90], 'Visible', cfg.visible);
    ax = axes('Parent', fig, 'Units', 'normalized', 'Position', [0.27 0.02 0.72 0.89], ...
              'Color', [0.07 0.09 0.13]);
    hold(ax, 'on');
    set(ax, 'XLim', [-1.5 1.5], 'YLim', [-1.35 1.35], 'ZLim', [-0.5 1.3], ...
            'XColor', [0.6 0.7 0.8], 'YColor', [0.6 0.7 0.8], 'ZColor', [0.6 0.7 0.8], ...
            'GridColor', [0.5 0.6 0.7], 'GridAlpha', 0.15, 'Projection', 'perspective', ...
            'AmbientLightColor', [0.45 0.45 0.5], 'FontSize', 8);
    daspect(ax, [1 1 1]);
    grid(ax, 'on');
    xlabel(ax, 'X [m]'); ylabel(ax, 'Y [m]'); zlabel(ax, 'Z [m]');
    title(ax, 'APEX_ROBOT  |  3D Robotic Manipulator - Intelligent Shape Sorting (Simulation)', ...
          'Color', [0.8 0.9 1], 'FontSize', 11);
    view(ax, [40 24]);
    light('Parent', ax, 'Style', 'infinite', 'Position', [2 -3 4],  'Color', [1.0 0.98 0.92]);
    light('Parent', ax, 'Style', 'infinite', 'Position', [-3 2 2],  'Color', [0.35 0.42 0.60]);

    buildEnvironment(ax, cfg);

    S.cfg      = cfg;
    S.fig      = fig;
    S.ax       = ax;
    S.robot    = buildRobot(ax, cfg);
    S.obj      = buildObjects(ax, cfg);
    S.slot     = buildStation(ax, cfg);
    S.ring     = buildRing(ax);
    S.hud      = buildHUD(fig, bg);

    [S.qHome, ok] = ikSolve(cfg.homeTip, cfg);
    if ~ok, error('RoboMind:scene', 'Home pose unreachable.'); end
    S.q           = S.qHome;
    S.gap         = cfg.gapIdle;
    S.carried     = 0;            
    S.carryOff    = [0 0 0];      
    S.watch       = 0;            
    S.selected    = 0;
    S.shape       = '---';
    S.numDetected = 0;
    S.numSorted   = 0;
    S.status      = 'SYSTEM INITIALIZING';
    S.operation   = 'Building scene';
    S.complete    = false;
    S.clock       = tic;

    S = applyPose(S);
    S = refreshHUD(S);
    drawnow;
end

function buildEnvironment(ax, ~)
    
    fz = -0.50;
    fl = struct('V', [-3 -3 fz; 3 -3 fz; 3 3 fz; -3 3 fz], 'F', [1 2 3; 1 3 4], 'smooth', false);
    addMesh(ax, fl, [0.10 0.11 0.14], 'matte');
    
    addMesh(ax, meshBox([0 0 -0.05], [2.8 2.5 0.10]), [0.26 0.30 0.36], 'matte');
    for sx = [-1 1]
        for sy = [-1 1]
            addMesh(ax, meshBox([1.3*sx 1.15*sy -0.30], [0.10 0.10 0.40]), [0.18 0.20 0.24], 'metal');
        end
    end
    
    X = []; Y = [];
    for x = -1.4:0.2:1.4, X = [X x x NaN]; Y = [Y -1.25 1.25 NaN]; end 
    for y = -1.2:0.2:1.2, X = [X -1.4 1.4 NaN]; Y = [Y y y NaN]; end   
    plot3(ax, X, Y, 0.002*ones(size(X)), 'Color', [0.36 0.40 0.47], 'LineWidth', 0.5);
    
    o = [-1.30 -1.10 0.003];  L = 0.30;
    plot3(ax, [o(1) o(1)+L], [o(2) o(2)],   [o(3) o(3)],   '-', 'Color', [1 0.3 0.3], 'LineWidth', 2.5);
    plot3(ax, [o(1) o(1)],   [o(2) o(2)+L], [o(3) o(3)],   '-', 'Color', [0.3 1 0.3], 'LineWidth', 2.5);
    plot3(ax, [o(1) o(1)],   [o(2) o(2)],   [o(3) o(3)+L], '-', 'Color', [0.4 0.6 1], 'LineWidth', 2.5);
    text(ax, o(1)+L+0.04, o(2), o(3), 'X', 'Color', [1 0.4 0.4], 'FontWeight', 'bold');
    text(ax, o(1), o(2)+L+0.04, o(3), 'Y', 'Color', [0.4 1 0.4], 'FontWeight', 'bold');
    text(ax, o(1), o(2), o(3)+L+0.04, 'Z', 'Color', [0.5 0.7 1], 'FontWeight', 'bold');
end

function R = buildRobot(ax, cfg)
    navy   = [0.05 0.10 0.28];  blue = [0.15 0.40 0.85];  cyan = [0.10 0.85 0.95];
    gold   = [0.90 0.72 0.20];  silver = [0.78 0.80 0.85]; dgrey = [0.18 0.19 0.21];

    % Nested transform chain: yaw -> shoulder -> elbow -> wrist -> fingers
    R.hYaw      = hgtransform('Parent', ax);
    R.hShoulder = hgtransform('Parent', R.hYaw);
    R.hElbow    = hgtransform('Parent', R.hShoulder);
    R.hWrist    = hgtransform('Parent', R.hElbow);
    R.hFingerL  = hgtransform('Parent', R.hWrist);
    R.hFingerR  = hgtransform('Parent', R.hWrist);

    % Static base (does not rotate) + blob shadow
    t = 2*pi*(0:39)' / 40;
    patch('Parent', ax, 'Faces', 1:40, 'Vertices', [0.34*cos(t) 0.34*sin(t) 0.004*ones(40, 1)], ...
          'FaceColor', 'k', 'FaceAlpha', 0.35, 'EdgeColor', 'none', 'FaceLighting', 'none');
    addMesh(ax, meshCylinder([0 0 0.06], 0.24, 0.12, 'z'), navy,   'metal');
    addMesh(ax, meshCylinder([0 0 0.13], 0.19, 0.02, 'z'), silver, 'metal');

    % Rotating turret (yaw frame): column + shoulder clevis plates
    addMesh(R.hYaw, meshCylinder([0 0 (0.14 + cfg.z0)/2], 0.15, cfg.z0 - 0.14, 'z'), navy, 'metal');
    addMesh(R.hYaw, meshBox([0  0.10 cfg.z0], [0.24 0.04 0.22]), [0.10 0.18 0.42], 'metal');
    addMesh(R.hYaw, meshBox([0 -0.10 cfg.z0], [0.24 0.04 0.22]), [0.10 0.18 0.42], 'metal');

    % Shoulder joint + main arm link 
    addMesh(R.hShoulder, meshCylinder([0 0 0], 0.085, 0.32, 'y'), gold, 'metal');
    addMesh(R.hShoulder, meshBox([cfg.L1/2 0 0], [cfg.L1 + 0.08, 0.12, 0.10]), blue, 'metal');

    % Elbow joint + secondary link 
    addMesh(R.hElbow, meshCylinder([0 0 0], 0.075, 0.16, 'y'), gold, 'metal');
    addMesh(R.hElbow, meshBox([cfg.L2/2 0 0], [cfg.L2 + 0.08, 0.10, 0.09]), cyan, 'metal');

    % Wrist joint + gripper palm 
    addMesh(R.hWrist, meshCylinder([0 0 0], 0.055, 0.13, 'y'), silver, 'metal');
    addMesh(R.hWrist, meshCylinder([0.03 0 0], 0.035, 0.06, 'x'), silver, 'metal');
    addMesh(R.hWrist, meshBox([0.085 0 0], [0.05 0.28 0.07]), dgrey, 'plastic');

    % Two fingers: box from x = 0.11 to x = Lg (tip), 0.02 thick in y
    fl = cfg.Lg - 0.11;
    addMesh(R.hFingerL, meshBox([0.11 + fl/2, 0, 0], [fl 0.02 0.05]), dgrey, 'plastic');
    addMesh(R.hFingerR, meshBox([0.11 + fl/2, 0, 0], [fl 0.02 0.05]), dgrey, 'plastic');
end

function objs = buildObjects(ax, cfg)
    for k = 1:numel(cfg.objects)
        c   = cfg.objects(k);
        g   = objectSpec(c.label);
        pos = [c.pos(1), c.pos(2), g.halfH];            
        hT  = hgtransform('Parent', ax, 'Matrix', Tr4(pos));
        addMesh(hT, g.mesh, c.color, 'plastic');
        
        t  = 2*pi*(0:31)' / 32;
        rs = 1.25 * g.footR;
        hS = hgtransform('Parent', ax, 'Matrix', Tr4([pos(1) pos(2) shadowZ(pos(1), pos(2), cfg)]));
        patch('Parent', hS, 'Faces', 1:32, 'Vertices', [rs*cos(t) rs*sin(t) zeros(32, 1)], ...
              'FaceColor', 'k', 'FaceAlpha', 0.35, 'EdgeColor', 'none', 'FaceLighting', 'none');
        o = struct('label', c.label, 'color', c.color, 'pos', pos, ...
                   'halfH', g.halfH, 'gripHalf', g.gripHalf, ...
                   'graspZ', max(g.halfH, cfg.minGraspZ), ...
                   'hT', hT, 'hShadow', hS, ...
                   'sorted', false, 'detected', false, 'slotIdx', 0);
        objs(k) = o; 
    end
end

function z = shadowZ(x, y, cfg)
    
    if abs(x) <= 0.85 && y >= 0.49 && y <= 0.91
        z = cfg.stationTopZ + 0.003;
    else
        z = 0.004;
    end
end

function slots = buildStation(ax, cfg)
    zt = cfg.stationTopZ;
    addMesh(ax, meshBox([0 cfg.stationY zt/2], [1.80 0.42 zt]), [0.16 0.18 0.22], 'metal');
    text(ax, 0, 0.95, 0.07, 'SHAPE SORTING STATION', 'Color', [0.5 0.9 1], ...
         'FontWeight', 'bold', 'FontSize', 10, 'HorizontalAlignment', 'center');
    for k = 1:numel(cfg.objects)
        label = cfg.objects(k).label;
        col   = cfg.objects(k).color;                 
        [P, typeName] = slotOutline(label);
        c = [cfg.slotX(k), cfg.stationY, zt];
        V = [P(:, 1) + c(1), P(:, 2) + c(2), (zt + 0.002)*ones(size(P, 1), 1)];
        patch('Parent', ax, 'Faces', 1:size(P, 1), 'Vertices', V, 'FaceColor', 0.45*col, ...
              'EdgeColor', col, 'LineWidth', 3, 'FaceLighting', 'none');
        text(ax, c(1), 0.50, 0.03, {upper(label), typeName}, 'Color', col, ...
             'FontWeight', 'bold', 'FontSize', 8, 'HorizontalAlignment', 'center', ...
             'VerticalAlignment', 'top', 'Interpreter', 'none');
        slots(k) = struct('label', label, 'color', col, 'center', c, 'poly', P, ...
                          'typeName', typeName); 
    end
end

function ring = buildRing(ax)
    n = 64;
    t = 2*pi*(0:n-1)';
    ring.c = cos(t);   ring.s = sin(t);
    V0 = [ring.c ring.s zeros(n, 1)];
    core = gobjects(1, 3);  glow = gobjects(1, 3);
    for i = 1:3
        core(i) = patch('Parent', ax, 'Faces', 1:n, 'Vertices', V0, 'FaceColor', 'none', ...
                        'EdgeColor', [0 1 1], 'LineWidth', 2.5, 'EdgeAlpha', 1, 'Visible', 'off');
        glow(i) = patch('Parent', ax, 'Faces', 1:n, 'Vertices', V0, 'FaceColor', 'none', ...
                        'EdgeColor', [0.4 1 1], 'LineWidth', 8, 'EdgeAlpha', 0.3, 'Visible', 'off');
    end
    ring.core = core;
    ring.glow = glow;
    ring.lock = patch('Parent', ax, 'Faces', 1:n, 'Vertices', V0, 'FaceColor', 'none', ...
                      'EdgeColor', [0 0.9 1], 'LineWidth', 2, 'EdgeAlpha', 0.9, 'Visible', 'off');
    ring.label = text(ax, 0, 0, 0.3, 'OBJECT DETECTED', 'Color', [0 1 1], 'FontWeight', 'bold', ...
                      'FontSize', 11, 'HorizontalAlignment', 'center', 'Visible', 'off', ...
                      'BackgroundColor', [0 0.12 0.20], 'EdgeColor', [0 1 1], 'Margin', 3);
    ring.active = false;
    ring.t0 = 0;
    ring.center = [0 0 0];
    ring.r0 = 0.04;  ring.r1 = 0.30;  ring.period = 1.2;
end

function hud = buildHUD(fig, bg)
    cy = [0.30 0.90 1.0];
    pnl = uipanel('Parent', fig, 'Units', 'normalized', 'Position', [0.005 0.01 0.26 0.98], ...
                  'BackgroundColor', bg, 'ForegroundColor', cy, 'BorderType', 'line', ...
                  'HighlightColor', [0.10 0.50 0.70]);
    mk = @(pos, str, fs, fw, fc) uicontrol('Parent', pnl, 'Style', 'text', 'Units', 'normalized', ...
         'Position', pos, 'String', str, 'FontSize', fs, 'FontWeight', fw, ...
         'ForegroundColor', fc, 'BackgroundColor', bg, 'HorizontalAlignment', 'left');
    mk([0.04 0.93 0.92 0.05], 'APEX_ROBOT', 18, 'bold', cy);
    mk([0.04 0.89 0.92 0.04], 'Robotic sorting - SIMULATION', 9, 'normal', [0.6 0.7 0.8]);
    names  = {'status', 'operation', 'selected', 'shape', 'detected', 'sorted', ...
              'progress', 'pos', 'joints', 'completion'};
    labels = {'ROBOT STATUS', 'CURRENT OPERATION', 'SELECTED OBJECT', 'DETECTED SHAPE', ...
              'OBJECTS DETECTED', 'OBJECTS SORTED', 'SORTING PROGRESS', ...
              'END-EFFECTOR XYZ [m]', 'JOINT ANGLES [deg]', 'COMPLETION'};
    top = 0.87;  rowH = 0.082;
    for i = 1:numel(names)
        y = top - i*rowH;
        mk([0.04 y + 0.043 0.92 0.03], labels{i}, 8, 'normal', [0.5 0.6 0.7]);
        hud.(names{i}) = mk([0.04 y 0.92 0.042], '', 10, 'bold', [0.95 0.98 1.0]);
    end
    mk([0.04 0.002 0.92 0.045], 'Shape ID = simulated label lookup (not a trained AI model).', ...
       7, 'normal', [0.9 0.7 0.3]);
    hud.banner = uicontrol('Parent', fig, 'Style', 'text', 'Units', 'normalized', ...
         'Position', [0.27 0.925 0.72 0.06], 'String', '', 'FontSize', 16, 'FontWeight', 'bold', ...
         'ForegroundColor', [0.2 0.95 1], 'BackgroundColor', [0.04 0.06 0.10]);
end


function obs = getObjectObservation(S, k)
   
    switch S.cfg.perceptionBackend
        case 'simulated'
            obs = struct('position', S.obj(k).pos, 'shapeHint', S.obj(k).label, ...
                         'source', 'SIMULATION');
        otherwise
            error('RoboMind:notImplemented', ...
                  'Perception back-end "%s" is not implemented in this version.', ...
                  S.cfg.perceptionBackend);
    end
end

function [near, dist] = proximityCheck(tip, objPos, detectDist)
    dist = norm(tip - objPos);
    near = dist <= detectDist;
end

function [label, info] = classifyShape(obs, cfg)
    
    switch cfg.classifierBackend
        case 'simulated'
            label = obs.shapeHint;
            info  = 'Simulated lookup (no trained model)';
        otherwise
            error('RoboMind:notImplemented', ...
                  'Classifier back-end "%s" is not implemented in this version.', ...
                  cfg.classifierBackend);
    end
end

function idx = findSlotForShape(slots, label)
    idx = find(strcmp({slots.label}, label), 1);
    if isempty(idx)
        error('RoboMind:noSlot', 'No sorting slot for shape "%s".', label);
    end
end

function k = selectNearestObject(objs, tip)
    idx = find(~[objs.sorted]);
    if isempty(idx), error('RoboMind:noObjects', 'No unsorted objects left.'); end
    P = vertcat(objs(idx).pos);
    d = sqrt(sum((P - tip).^2, 2));
    [~, m] = min(d);
    k = idx(m);
end

function [ok, err] = verifyPlacement(finalPos, slot, halfH, tol)
    target = [slot.center(1), slot.center(2), slot.center(3) + halfH];
    err = norm(finalPos - target);
    inside = inpolygon(finalPos(1), finalPos(2), ...
                       slot.poly(:, 1) + slot.center(1), slot.poly(:, 2) + slot.center(2));
    ok = (err <= tol) && inside;
end


function S = renderFrame(S)
    if ~isgraphics(S.fig)
        error('RoboMind:figureClosed', 'Figure closed by user.');
    end
    S = applyPose(S);
    
    if S.watch > 0 && ~S.obj(S.watch).detected
        near = proximityCheck(fkTip(S.q, S.cfg), S.obj(S.watch).pos, S.cfg.detectDist);
        if near, S = triggerDetection(S, S.watch); end
    end
    S = updateDetectionAnim(S);
    p = fkTip(S.q, S.cfg);
    set(S.hud.pos,    'String', sprintf('%6.3f  %6.3f  %6.3f', p));
    set(S.hud.joints, 'String', sprintf('%6.1f %6.1f %6.1f %6.1f', S.q * 180/pi));
    drawnow;
end

function S = applyPose(S)
    F = fkFrames(S.q, S.cfg);
    R = S.robot;
    set(R.hYaw,      'Matrix', F.yaw);
    set(R.hShoulder, 'Matrix', F.shoulder);
    set(R.hElbow,    'Matrix', F.elbow);
    set(R.hWrist,    'Matrix', F.wrist);
    t = S.cfg.fingerHalfT;
    set(R.hFingerL, 'Matrix', Tr4([0  (S.gap + t) 0]));
    set(R.hFingerR, 'Matrix', Tr4([0 -(S.gap + t) 0]));
    if S.carried > 0
        S = setObjPos(S, S.carried, fkTip(S.q, S.cfg) + S.carryOff);
    end
end

function S = setObjPos(S, k, pos)
    S.obj(k).pos = pos;
    set(S.obj(k).hT, 'Matrix', Tr4(pos));
    set(S.obj(k).hShadow, 'Matrix', Tr4([pos(1) pos(2) shadowZ(pos(1), pos(2), S.cfg)]));
end

function S = runMotion(S, mode, target, dur)
   
    dur = dur / S.cfg.speed;
    q0  = S.q;
    p0  = fkTip(q0, S.cfg);
    if strcmp(mode, 'joint'), q1 = alignYaw(target, q0); end
    t0 = tic;
    while true
        if dur > 0, tau = min(1, toc(t0) / dur); else, tau = 1; end
        s = 10*tau^3 - 15*tau^4 + 6*tau^5;
        if strcmp(mode, 'joint')
            q = q0 + (q1 - q0) * s;
        else
            [q, ok] = ikSolve(p0 + (target - p0) * s, S.cfg);
            if ~ok, error('RoboMind:unreachable', 'Linear move left the workspace.'); end
            q = alignYaw(q, q0);
        end
        S.q = q;
        S = renderFrame(S);
        if tau >= 1, break; end
    end
end

function S = runGripper(S, gapEnd, dur)
    dur = dur / S.cfg.speed;
    g0  = S.gap;
    t0  = tic;
    while true
        if dur > 0, tau = min(1, toc(t0) / dur); else, tau = 1; end
        S.gap = g0 + (gapEnd - g0) * tau*tau*(3 - 2*tau);
        S = renderFrame(S);
        if tau >= 1, break; end
    end
end

function S = waitAnim(S, dur)
    dur = dur / S.cfg.speed;
    t0  = tic;
    while true
        S = renderFrame(S);
        if toc(t0) >= dur, break; end
    end
end

function S = triggerDetection(S, k)
    S.obj(k).detected = true;
    S.numDetected = S.numDetected + 1;
    p = S.obj(k).pos;
    R = S.ring;
    R.active = true;
    R.center = p;
    R.t0 = toc(S.clock);
    n = numel(R.c);
    set(R.lock, 'Vertices', [p(1) + 0.11*R.c, p(2) + 0.11*R.s, 0.007*ones(n, 1)], 'Visible', 'on');
    set(R.label, 'Position', [p(1) p(2) p(3) + 0.32], 'Visible', 'on');
    S.ring = R;
    S = setStatus(S, 'OBJECT DETECTED', 'Proximity sensor triggered');
end

function S = updateDetectionAnim(S)
    R = S.ring;
    if ~R.active, return; end
    t = toc(S.clock) - R.t0;
    n = numel(R.c);
    for i = 1:3
        u = t / R.period - (i - 1) / 3;
        if u < 0
            set([R.core(i) R.glow(i)], 'Visible', 'off');
            continue;
        end
        ph  = mod(u, 1);
        rad = R.r0 + (R.r1 - R.r0) * ph;
        a   = 1 - ph;
        V   = [R.center(1) + rad*R.c, R.center(2) + rad*R.s, 0.007*ones(n, 1)];
        set(R.core(i), 'Vertices', V, 'EdgeAlpha', max(a, 0.05), 'Visible', 'on');
        set(R.glow(i), 'Vertices', V, 'EdgeAlpha', max(0.35*a, 0.02), 'Visible', 'on');
    end
end

function S = stopDetectionAnim(S)
    R = S.ring;
    R.active = false;
    set([R.core R.glow R.lock R.label], 'Visible', 'off');
    S.ring = R;
end

function S = setStatus(S, status, operation)
    S.status = status;
    S.operation = operation;
    S = refreshHUD(S);
end

function S = refreshHUD(S)
    h = S.hud;
    n = numel(S.obj);
    if S.selected > 0, sel = upper(S.obj(S.selected).label); else, sel = 'NONE'; end
    f = round(10 * S.numSorted / n);
    prog = sprintf('[%s%s] %d/%d (%d%%)', repmat('#', 1, f), repmat('-', 1, 10 - f), ...
                   S.numSorted, n, round(100 * S.numSorted / n));
    if S.complete, comp = 'COMPLETE - ROBOT AT HOME'; else, comp = 'IN PROGRESS'; end
    set(h.status,     'String', S.status);
    set(h.operation,  'String', S.operation);
    set(h.selected,   'String', sel);
    set(h.shape,      'String', S.shape);
    set(h.detected,   'String', sprintf('%d / %d', S.numDetected, n));
    set(h.sorted,     'String', sprintf('%d / %d', S.numSorted, n));
    set(h.progress,   'String', prog);
    set(h.completion, 'String', comp);
    if S.complete
        set(h.banner, 'String', S.cfg.finalMsg, 'ForegroundColor', [0.3 1 0.5]);
    else
        set(h.banner, 'String', S.status, 'ForegroundColor', [0.2 0.95 1]);
    end
end


function S = runCommandLoop(S)
    
    fprintf('\n==============================================\n');
    fprintf('              APEX_ROBOT READY\n');
    fprintf('Type "help" to see available commands.\n');
    fprintf('==============================================\n\n');

    while isgraphics(S.fig)
        cmd = input('Apex_Robot > ', 's');
        cmd = lower(strtrim(cmd));
        if isempty(cmd), continue; end

        if any(strcmp(cmd, {'help','commands','?'}))
            fprintf('\nCOMMANDS YOU CAN ENTER:\n');
            fprintf('  sort all          - sort every remaining object\n');
            fprintf('  sort cube         - sort the cube\n');
            fprintf('  sort sphere       - sort the sphere\n');
            fprintf('  sort cylinder     - sort the cylinder\n');
            fprintf('  sort triangular prism - sort the triangular prism\n');
            fprintf('  sort circular disk - sort the circular disk\n');
            fprintf('  pick cube         - same as sort cube\n');
            fprintf('  pick up the cube  - natural phrasing also accepted\n');
            fprintf('  home              - return robot to home position\n');
            fprintf('  status            - show sorting progress\n');
            fprintf('  help              - show this command list\n');
            fprintf('  exit              - return home and close command session\n\n');

        elseif any(strcmp(cmd, {'status','show status','progress'}))
            fprintf('Status: %s\n', S.status);
            fprintf('Sorted: %d / %d objects\n', S.numSorted, numel(S.obj));
            for j = 1:numel(S.obj)
                if S.obj(j).sorted, state = 'SORTED'; else, state = 'WAITING'; end
                fprintf('  %-20s : %s\n', S.obj(j).label, state);
            end

        elseif any(strcmp(cmd, {'home','go home','return home','move home'}))
            S = moveRobotHome(S);

        elseif any(strcmp(cmd, {'exit','quit','stop'}))
            S = moveRobotHome(S);
            fprintf('Command control stopped.\n');
            break;

        elseif startsWith(cmd, 'sort ') || startsWith(cmd, 'pick ')
            if startsWith(cmd, 'pick up ')
                requested = strtrim(cmd(numel('pick up ')+1:end));
            else
                parts = strsplit(cmd);
                requested = strtrim(strjoin(parts(2:end), ' '));
            end
            requested = regexprep(requested, '^the\s+', '');
            requested = regexprep(requested, '\s+object$', '');
            if any(strcmp(requested, {'all','everything','all objects'}))
                while any(~[S.obj.sorted])
                    k = selectNearestObject(S.obj, fkTip(S.q, S.cfg));
                    S.selected = k;
                    S = refreshHUD(S);
                    fprintf('Sorting %s...\n', S.obj(k).label);
                    S = pickAndPlace(S, k);
                end
                S = moveRobotHome(S);
                fprintf('All objects sorted. Robot returned home.\n');
            else
                labels = {S.obj.label};
                labels = cellfun(@lower, labels, 'UniformOutput', false);
                key = strrep(requested, ' ', '');
                normalized = cellfun(@(x) strrep(x, ' ', ''), labels, 'UniformOutput', false);
                hit = find(strcmp(normalized, key), 1);
                if isempty(hit)
                    fprintf('I do not recognize "%s". Type "help" for valid object names.\n', requested);
                elseif S.obj(hit).sorted
                    fprintf('%s is already sorted.\n', S.obj(hit).label);
                else
                    S.selected = hit;
                    S = refreshHUD(S);
                    fprintf('Executing command: sort %s\n', S.obj(hit).label);
                    S = pickAndPlace(S, hit);
                    fprintf('%s sorted successfully.\n', S.obj(hit).label);
                end
            end

        else
            fprintf('Command not recognized: "%s". Type "help".\n', cmd);
        end
    end
end

function S = moveRobotHome(S)
    S = setStatus(S, 'RETURNING HOME', 'Moving to home position');
    S = runMotion(S, 'joint', S.qHome, 2.2);
    S = runGripper(S, S.cfg.gapIdle, 0.5);
    if norm(fkTip(S.q, S.cfg) - S.cfg.homeTip) > 1e-3
        error('RoboMind:home', 'Robot did not reach the home position.');
    end
    if all([S.obj.sorted])
        S.complete = true;
        S = setStatus(S, 'SORTING COMPLETED', S.cfg.finalMsg);
    else
        S = setStatus(S, 'ROBOT READY', 'Waiting for the next command');
    end
end

function S = runSimulation(S)
    cfg = S.cfg;
    S = setStatus(S, 'SYSTEM INITIALIZING', 'Self-tests passed, scene loaded');
    S = waitAnim(S, 1.2);
    S = setStatus(S, 'ROBOT READY', 'Idle at home position');
    S = waitAnim(S, 1.2);

    while any(~[S.obj.sorted])
        S = setStatus(S, 'SEARCHING FOR OBJECT', 'Selecting nearest unsorted object');
        S = waitAnim(S, 0.5);
        k = selectNearestObject(S.obj, fkTip(S.q, cfg));
        S.selected = k;
        S = refreshHUD(S);
        S = pickAndPlace(S, k);
    end

    S = setStatus(S, 'RETURNING HOME', 'Moving to home position');
    S = runMotion(S, 'joint', S.qHome, 2.2);
    S = runGripper(S, cfg.gapIdle, 0.5);
    if norm(fkTip(S.q, cfg) - cfg.homeTip) > 1e-3
        error('RoboMind:home', 'Robot did not reach the home position.');
    end
    S.complete = true;
    S = setStatus(S, 'SORTING COMPLETED', cfg.finalMsg);
    S = waitAnim(S, 0.5);
end

function S = pickAndPlace(S, k)
    cfg = S.cfg;
    o   = S.obj(k);
    p   = o.pos;

    
    S = setStatus(S, 'SEARCHING FOR OBJECT', 'Approaching object from above');
    [qHover, ok] = ikSolve([p(1) p(2) cfg.safeZ], cfg);
    if ~ok, error('RoboMind:unreachable', 'Hover pose above %s unreachable.', o.label); end
    S.watch = k;                                  
    S = runMotion(S, 'joint', qHover, 2.2);
    S.watch = 0;

    
    if ~S.obj(k).detected, S = triggerDetection(S, k); end
    S = setStatus(S, 'OBJECT DETECTED', 'Ready for classification');
    S = waitAnim(S, 1.4);

    
    obs = getObjectObservation(S, k);
    [label, info] = classifyShape(obs, cfg);
    slotIdx = findSlotForShape(S.slot, label);
    S.obj(k).slotIdx = slotIdx;
    S.shape = upper(label);
    S = setStatus(S, 'CLASSIFICATION COMPLETE', info);
    S = waitAnim(S, 1.0);

    
    S = setStatus(S, 'PICKING OBJECT', 'Opening gripper');
    S = runGripper(S, cfg.gapOpen, 0.6);
    S = setStatus(S, 'PICKING OBJECT', 'Lowering gripper');
    S = runMotion(S, 'linear', [p(1) p(2) o.graspZ], 1.4);
    S = setStatus(S, 'PICKING OBJECT', 'Closing gripper');
    S = runGripper(S, o.gripHalf, 0.6);
    S.carryOff = S.obj(k).pos - fkTip(S.q, cfg);  
    S.carried  = k;
    S = stopDetectionAnim(S);

    
    S = setStatus(S, 'PICKING OBJECT', 'Lifting object');
    S = runMotion(S, 'linear', [p(1) p(2) cfg.safeZ], 1.2);

    
    c = S.slot(slotIdx).center;
    [qAbove, ok] = ikSolve([c(1) c(2) cfg.safeZ], cfg);
    if ~ok, error('RoboMind:unreachable', 'Slot %d unreachable.', slotIdx); end
    S = setStatus(S, 'TRANSPORTING OBJECT', ['Moving to ' S.slot(slotIdx).typeName]);
    S = runMotion(S, 'joint', qAbove, 2.4);

    
    S = setStatus(S, 'PLACING OBJECT', 'Lowering object into slot');
    targetObj = [c(1) c(2) c(3) + o.halfH];
    S = runMotion(S, 'linear', targetObj - S.carryOff, 1.4);

    
    S = setStatus(S, 'PLACING OBJECT', 'Releasing object');
    S.carried = 0;
    S = setObjPos(S, k, targetObj);               
    S = runGripper(S, cfg.gapOpen, 0.6);

   
    [okV, err] = verifyPlacement(S.obj(k).pos, S.slot(slotIdx), o.halfH, cfg.verifyTol);
    if ~okV
        error('RoboMind:placement', 'Placement of %s failed verification.', o.label);
    end
    S = setStatus(S, 'PLACEMENT VERIFIED', sprintf('Position error %.2f mm', err*1000));
    S = waitAnim(S, 0.7);

    
    S = runMotion(S, 'linear', [c(1) c(2) cfg.safeZ], 1.0);
    S = runGripper(S, cfg.gapIdle, 0.4);

 
    S.obj(k).sorted = true;
    S.numSorted = S.numSorted + 1;
    S.selected  = 0;
    S.shape     = '---';
    S = refreshHUD(S);
end
