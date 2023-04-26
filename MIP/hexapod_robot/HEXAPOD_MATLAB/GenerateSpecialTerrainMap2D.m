function SpecialTerrainMap2D=GenerateSpecialTerrainMap2D() %生成特殊的地形 调用GenerateTerrainMap2D子函数

    SpecialTerrainMap2D=[];
    for i=-2:0.3:10
        for j=-2:0.3:2
            SpecialTerrainMap2D=[SpecialTerrainMap2D;i j -0.5];
        end
    end

end
