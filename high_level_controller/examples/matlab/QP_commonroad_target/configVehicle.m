function cfg = configVehicle

       cfg.Hu = 7; % Stellhorizont
       cfg.Hp = 8; % (oberer) Praediktionshorizont
       cfg.dt = 0.25; % Abtastzeit in Sekunden
       cfg.dsafeVehicles = 0.2; % Sicherheitsabstand
       cfg.minSpeed = 0.02; % minimale Geschwindigkeit
       cfg.speed_target = 1; % Reisegeschwindigkeit

end