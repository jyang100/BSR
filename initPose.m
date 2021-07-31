function poses = initPose(N,D,K)
    %N为机器人的个数，D为机器人的半径，K为两机器人之间的距离（D的倍数）
    b=D*ones(1,N);  % 生成半径为D的数组，N个
    S=[N,N];                              %**区域大小**
    poses = zeros(3,N);
    poses(:,1) = [(N-1)/2,(N-1)/2,pi/2]; % Leader position
    for ii=2:N
        p=b(ii)+(S-2*b(ii)).*rand(1,2);   %nNew Random point
        A = poses(:, ~all(poses==0));
        while any((A(1,:)-p(1)).^2+(A(2,:)-p(2)).^2<K*K*D*D)   % Evaluate the distance
            p=b(ii)+(S-2*b(ii)).*rand(1,2);
        end
        poses(:,ii) = [p'; 0.5*pi*rand(1,1)];    %save new point
    end
end