function kmlStruct = kml2struct(kmlFile,force_order_cw)
  % kmlStruct = kml2struct(kmlFile)
  %
  % Import a .kml file as a vector array of shapefile structs, with Geometry, Name,
  % Description, Lon, Lat, and BoundaryBox fields.  Structs may contain a mix
  % of points, lines, and polygons.
  %
  % .kml files with folder structure will not be presented as such, but will
  % appear as a single vector array of structs.
  %
  %
  
  [FID msg] = fopen(kmlFile,'rt');
  
  if FID<0
    error(msg)
  end
  
  txt = fread(FID,'uint8=>char')';
  fclose(FID);
  
  expr = '<Placemark.+?>.+?</Placemark>';
  
  objectStrings = regexp(txt,expr,'match');
  
  Nos = length(objectStrings);
  
  for ii = 1:Nos
    % Find Object Name Field
    bucket = regexp(objectStrings{ii},'<name.*?>.+?</name>','match');
    if isempty(bucket)
      name = 'undefined';
    else
      % Clip off flags
      name = regexprep(bucket{1},'<name.*?>\s*','');
      name = regexprep(name,'\s*</name>','');
    end
    
    % Find Object Description Field
    bucket = regexp(objectStrings{ii},'<description.*?>.+?</description>','match');
    if isempty(bucket)
      desc = '';
    else
      % Clip off flags
      desc = regexprep(bucket{1},'<description.*?>\s*','');
      desc = regexprep(desc,'\s*</description>','');
    end
    
    geom = 0;
    % Identify Object Type
    if ~isempty(regexp(objectStrings{ii},'<Point', 'once'))
      geom = 1;
    elseif ~isempty(regexp(objectStrings{ii},'<LineString', 'once'))
      geom = 2;
    elseif ~isempty(regexp(objectStrings{ii},'<Polygon', 'once'))
      geom = 3;
    end
    
    switch geom
      case 1
        geometry = 'Point';
      case 2
        geometry = 'Line';
      case 3
        geometry = 'Polygon';
      otherwise
        geometry = '';
    end
    
    % Find Coordinate Field
    bucket = regexp(objectStrings{ii},'<coordinates.*?>.+?</coordinates>','match');
    % Clip off flags
    coordStr = regexprep(bucket{1},'<coordinates.*?>(\s+)*','');
    coordStr = regexprep(coordStr,'(\s+)*</coordinates>','');
    % Split coordinate string by commas or white spaces, and convert string
    % to doubles
    coordMat = str2double(regexp(coordStr,'[,\s]+','split'));
    % Rearrange coordinates to form an x-by-3 matrix
    [m,n] = size(coordMat);
    coordMat = reshape(coordMat,3,m*n/3)';
    
    % check if polyline or polygon is defined clockwise
    CLOCK_WISE = false;
    if length(coordMat(:,1)) > 1 % if it is a line
      %
%       disp([coordMat(1,1),coordMat(1,2)])
%       disp([coordMat(end,1),coordMat(end,2)])
      if force_order_cw % voglio clock wise:
        if ~ispolycw(coordMat(:,1),coordMat(:,2)) % it is not cw quindi flip() !!! ATTENZIONE, DA SISTEMARE CON SOSTITUTO OPENSOURCE DI ISPOLYCW !!!
          tmp = coordMat(:,1); coordMat(:,1)=flip(tmp);
          tmp = coordMat(:,2); coordMat(:,2)=flip(tmp);
          tmp = coordMat(:,3); coordMat(:,3)=flip(tmp);
        end
        CLOCK_WISE = true;
      else % I want  conterclowise
        if ispolycw(coordMat(:,1),coordMat(:,2))  !!! ATTENZIONE, DA SISTEMARE CON SOSTITUTO OPENSOURCE DI ISPOLYCW !!!
          tmp = coordMat(:,1); coordMat(:,1)=flip(tmp);
          tmp = coordMat(:,2); coordMat(:,2)=flip(tmp);
          tmp = coordMat(:,3); coordMat(:,3)=flip(tmp);
       end
        CLOCK_WISE = false;
      end
%       disp([coordMat(1,1),coordMat(1,2)])
%       disp([coordMat(end,1),coordMat(end,2)])
    end
    
    % define polygon in clockwise direction, and terminate
    %  [Lat, Lon] = poly2ccw(coordMat(:,2),coordMat(:,1)); % originale
    % [Lon, Lat] = poly2ccw(coordMat(:,1),coordMat(:,2));
    %idx_ccw = find(coordMat(:,2)==Lat);
    %Alt = coordMat(idx_ccw,3);
    
    % ASSUMO SIANO GIA' ORDINATI SECONDO L'ORDINE DI PERCORRENZA
    Lon = coordMat(:,1);
    Lat = coordMat(:,2);
    Alt = coordMat(:,3);
    
    % COMMENTO LE SEGUENTI
    %     if geom==3
    %         Lon = [Lon;NaN];
    %         Lat = [Lat;NaN];
    %         Alt = [Alt;NaN];
    %     end
    
    % Create structure
    kmlStruct(ii).ClockWise  = CLOCK_WISE;
    kmlStruct(ii).Geometry    = geometry;
    kmlStruct(ii).Name        = name;
    kmlStruct(ii).Description = desc;
    kmlStruct(ii).Lon         = Lon;
    kmlStruct(ii).Lat         = Lat;
    kmlStruct(ii).Alt         = Alt;
    kmlStruct(ii).BoundingBox = [[min(Lon) min(Lat);max(Lon) max(Lat)]];
  end