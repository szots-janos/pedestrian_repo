function variable_plots(n_cases, s_, prefix, suffix, variable_description, variable_short, loc)
results = {zeros(100, n_cases), zeros(100, n_cases), zeros(100, n_cases)};
labels = ["APF", "SVO", "EBG"];
collisions = {zeros(1, n_cases), zeros(1, n_cases), zeros(1, n_cases)};
max_time = 0;
for i = 1:n_cases
    for j = 1:3
        label = labels(j);
        temp = load(replace(label + prefix + num2str(s_(i)) + suffix, ".", ""));
        results{j}(:, i) = temp.results;
        results{j}(temp.results < 0, i) = nan;
        collisions{j}(i) = sum(temp.results < 0);
        max_time = max(max_time, max(temp.results));
    end
end
%%
figure(1), clf, hold on
for j = 1:3
    label = labels(j);
    plot(s_, collisions{j}, '*-', DisplayName=label)
end
xticks(s_);
xlabel(variable_description)
ylabel("Collisions [-]")
legend('Location', 'Best');
set(gcf, 'Units', 'Inches', 'Position', [0, 0, 5, 3.8], 'PaperUnits', 'Inches', 'PaperSize', [5, 3.8])
exportgraphics(gcf, pwd + "/Figures/collisions_" + variable_short + ".pdf");
%%
figure(2), clf, hold on
for j = 1:3
    label = labels(j);
    plot(s_, nanmean(results{j}), '*-', DisplayName=label+" average")
    set(gca,'ColorOrderIndex', get(gca, 'ColorOrderIndex') - 1)
    plot(s_, nanmedian(results{j}), 'o--', DisplayName=label+" median")
end
xticks(s_);
xlabel(variable_description)
ylabel("Time to reach goal [s]")
legend('Location', loc);
set(gcf, 'Units', 'Inches', 'Position', [0, 0, 5, 3.8], 'PaperUnits', 'Inches', 'PaperSize', [5, 3.8])
exportgraphics(gcf, pwd + "/Figures/times_" + variable_short + ".pdf");
end