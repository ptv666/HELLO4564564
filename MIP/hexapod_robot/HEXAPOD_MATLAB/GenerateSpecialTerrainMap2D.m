function SpecialTerrainMap2D=GenerateSpecialTerrainMap2D() %��������ĵ��� ����GenerateTerrainMap2D�Ӻ���

    SpecialTerrainMap2D=[];
    for i=-2:0.3:10
        for j=-2:0.3:2
            SpecialTerrainMap2D=[SpecialTerrainMap2D;i j -0.5];
        end
    end

end
