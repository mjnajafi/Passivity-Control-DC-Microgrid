function status = verifyControlConditions(DG, Line, B_il)
    numOfDGs = size(B_il,1);
    numOfLines = size(B_il,2);
    status = true;

    fprintf('\n=== Verifying Control Design Conditions ===\n');

    % 1. Check DG Passivity Properties
    fprintf('\n1. DG Passivity Parameters:\n');
    for i = 1:numOfDGs
        fprintf('DG %d:\n', i);
        fprintf('  nu: %.6e\n', DG{i}.nu);
        fprintf('  rho: %.6e\n', DG{i}.rho);
        fprintf('  gammaTilde0: %.6e\n', DG{i}.gammaTilde0);
        
        % Check basic conditions from paper
        if DG{i}.rho <= 0
            fprintf('WARNING: DG %d rho should be positive\n', i);
            status = false;
        end
        if DG{i}.nu >= 0
            fprintf('WARNING: DG %d nu should be negative\n', i);
            status = false;
        end
    end

    % 2. Check Line Passivity Properties
    fprintf('\n2. Line Passivity Parameters:\n');
    for l = 1:numOfLines
        fprintf('Line %d:\n', l);
        fprintf('  nu: %.6e\n', Line{l}.nu);
        fprintf('  rho: %.6e\n', Line{l}.rho);
        
        if Line{l}.rho <= 0
            fprintf('WARNING: Line %d rho should be positive\n', l);
            status = false;
        end
    end

    % % 3. Check Interconnection Conditions
    % fprintf('\n3. Interconnection Conditions:\n');
    % for i = 1:numOfDGs
    %     for l = 1:numOfLines
    %         if B_il(i,l) ~= 0
    %             Ct = DG{i}.C;
    %             cond1 = Line{l}.rho > -DG{i}.nu/(Ct^2);
    %             if ~cond1
    %                 fprintf('WARNING: DG %d - Line %d fails interconnection condition 1\n', i, l);
    %                 status = false;
    %             end
    %         end
    %     end
    % end

     % 3. Check Interconnection Conditions with detailed output
    fprintf('\n3. Interconnection Conditions:\n');
    for i = 1:numOfDGs
        for l = 1:numOfLines
            if B_il(i,l) ~= 0
                Ct = DG{i}.C;
                required_rho = -DG{i}.nu/(Ct^2);
                actual_rho = Line{l}.rho;
                
                fprintf('DG %d - Line %d:\n', i, l);
                fprintf('  Required Line rho > %.4e\n', required_rho);
                fprintf('  Actual Line rho = %.4e\n', actual_rho);
                fprintf('  Ratio = %.4e\n', actual_rho/required_rho);
                
                if actual_rho <= required_rho
                    fprintf('  WARNING: Condition failed\n');
                    status = false;
                end
            end
        end
    end

    % Final status message using if-else instead of ternary operator
    if status
        fprintf('\nVerification PASSED\n');
    else
        fprintf('\nVerification FAILED\n');
    end
end