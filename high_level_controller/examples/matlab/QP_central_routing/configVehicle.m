function cfg = configVehicle

       cfg.Hu = 6; % Stellhorizont
       cfg.Hp = 8; % (oberer) Praediktionshorizont
       cfg.dt = 0.25; % Abtastzeit in Sekunden
       cfg.dsafeVehicles = 0.15; % Sicherheitsabstand
       cfg.minSpeed = 0.3; % minimale Geschwindigkeit
       cfg.speed_target = 1; % Reisegeschwindigkeit
end
