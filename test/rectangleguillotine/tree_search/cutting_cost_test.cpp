#include "packingsolver/rectangleguillotine/instance_builder.hpp"
#include "rectangleguillotine/tree_search.hpp"

#include "treesearchsolver/iterative_beam_search_2.hpp"

#include "packingsolver/algorithms/common.hpp"

#include <cstdio>
#include <fstream>
#include <gtest/gtest.h>

using namespace packingsolver;
using namespace packingsolver::rectangleguillotine;

TEST(RectangleGuillotineBranchingScheme, CuttingCostSingleItem)
{
    /**
     * A single item exactly filling the bin: no cuts needed, only the bin
     * cost applies.
     */
    InstanceBuilder instance_builder;
    instance_builder.set_objective(Objective::BinPackingCuttingCost);
    instance_builder.set_number_of_stages(3);
    instance_builder.set_cut_type(CutType::NonExact);
    instance_builder.set_item_type_stack_id(instance_builder.add_item_type(100, 100, false), 0);
    BinTypeId bin_type_id = instance_builder.add_bin_type(100, 100);

    instance_builder.set_bin_type_cost(bin_type_id, 1);
    instance_builder.set_bin_type_copies(bin_type_id, -1);
    instance_builder.set_fixed_cutting_cost(0, 10);
    instance_builder.set_variable_cutting_cost(0, 1);
    for (Counter stage_id = 1; stage_id <= 4; ++stage_id) {
        instance_builder.set_fixed_cutting_cost(stage_id, 5);
        instance_builder.set_variable_cutting_cost(stage_id, 1);
    }
    Instance instance = instance_builder.build();

    BranchingScheme::Parameters branching_scheme_parameters;
    BranchingScheme branching_scheme(instance, branching_scheme_parameters);
    auto output = treesearchsolver::iterative_beam_search_2(branching_scheme);
    Solution solution = branching_scheme.to_solution(output.solution_pool.best());

    EXPECT_TRUE(solution.full());
    // Bin cost only: fixed + variable * area = 10 + 1 * (100 * 100).
    EXPECT_EQ(solution.cutting_cost(), 10 + 1 * 100 * 100);
}

TEST(RectangleGuillotineBranchingScheme, CuttingCostTwoColumns)
{
    /**
     * Two items exactly filling one column each: one real 1-cut splits the
     * bin into the two columns, each item exactly fills its column (no
     * further 2-cut/3-cut needed).
     *
     * |-----------------|-----------------|
     * |                 |                 |
     * |        0        |        1        | 100
     * |                 |                 |
     * |-----------------|-----------------|
     *                 100               200
     */
    InstanceBuilder instance_builder;
    instance_builder.set_objective(Objective::BinPackingCuttingCost);
    instance_builder.set_number_of_stages(3);
    instance_builder.set_cut_type(CutType::NonExact);
    ItemTypeId item_type_id = instance_builder.add_item_type(100, 100, false);
    instance_builder.set_item_type_stack_id(item_type_id, 0);
    instance_builder.set_item_type_copies(item_type_id, 2);
    BinTypeId bin_type_id = instance_builder.add_bin_type(200, 100);

    instance_builder.set_bin_type_cost(bin_type_id, 1);
    instance_builder.set_bin_type_copies(bin_type_id, -1);
    instance_builder.set_fixed_cutting_cost(0, 10);
    instance_builder.set_variable_cutting_cost(0, 1);
    instance_builder.set_fixed_cutting_cost(1, 5);
    instance_builder.set_variable_cutting_cost(1, 1);
    for (Counter stage_id = 2; stage_id <= 4; ++stage_id) {
        instance_builder.set_fixed_cutting_cost(stage_id, 5);
        instance_builder.set_variable_cutting_cost(stage_id, 1);
    }
    Instance instance = instance_builder.build();

    BranchingScheme::Parameters branching_scheme_parameters;
    BranchingScheme branching_scheme(instance, branching_scheme_parameters);
    auto output = treesearchsolver::iterative_beam_search_2(branching_scheme);
    Solution solution = branching_scheme.to_solution(output.solution_pool.best());

    EXPECT_TRUE(solution.full());
    // Bin cost: 10 + 1 * (200 * 100) = 20010.
    // One real 1-cut: 5 + 1 * 100 (bin height) = 105.
    // No 2-cut/3-cut: each item exactly fills its column.
    EXPECT_EQ(solution.cutting_cost(), 20010 + 105);
}

TEST(RectangleGuillotineBranchingScheme, CuttingCostUnsetIsZero)
{
    /**
     * None of the cutting costs are set: the instance builder must not
     * throw, and the resulting cutting cost must be 0.
     */
    InstanceBuilder instance_builder;
    instance_builder.set_objective(Objective::BinPackingCuttingCost);
    instance_builder.set_number_of_stages(3);
    instance_builder.set_cut_type(CutType::NonExact);
    instance_builder.set_item_type_stack_id(instance_builder.add_item_type(100, 100, false), 0);
    BinTypeId bin_type_id = instance_builder.add_bin_type(100, 100);

    instance_builder.set_bin_type_cost(bin_type_id, 1);
    instance_builder.set_bin_type_copies(bin_type_id, -1);
    Instance instance = instance_builder.build();

    BranchingScheme::Parameters branching_scheme_parameters;
    BranchingScheme branching_scheme(instance, branching_scheme_parameters);
    auto output = treesearchsolver::iterative_beam_search_2(branching_scheme);
    Solution solution = branching_scheme.to_solution(output.solution_pool.best());

    EXPECT_TRUE(solution.full());
    EXPECT_EQ(solution.cutting_cost(), 0);
}

TEST(RectangleGuillotineBranchingScheme, CuttingCostWasteCost)
{
    /**
     * Two items, each leaving waste on the side of its own bin: only the
     * waste cost applies (bin/cut costs left at their default of 0). The
     * trailing open strip of the very last bin is "residual" (not yet
     * consumed stock) rather than waste, so only the first bin's leftover
     * is actually charged.
     *
     * |-------------|---------|
     * |             |         |
     * |      0      |  waste  | 100
     * |             |         |
     * |-------------|---------|
     *              100       150
     */
    InstanceBuilder instance_builder;
    instance_builder.set_objective(Objective::BinPackingCuttingCost);
    instance_builder.set_number_of_stages(3);
    instance_builder.set_cut_type(CutType::NonExact);
    ItemTypeId item_type_id = instance_builder.add_item_type(100, 100, false);
    instance_builder.set_item_type_stack_id(item_type_id, 0);
    instance_builder.set_item_type_copies(item_type_id, 2);
    BinTypeId bin_type_id = instance_builder.add_bin_type(150, 100);

    instance_builder.set_bin_type_cost(bin_type_id, 1);
    instance_builder.set_bin_type_copies(bin_type_id, -1);
    instance_builder.set_waste_cost(2);
    Instance instance = instance_builder.build();

    BranchingScheme::Parameters branching_scheme_parameters;
    BranchingScheme branching_scheme(instance, branching_scheme_parameters);
    auto output = treesearchsolver::iterative_beam_search_2(branching_scheme);
    Solution solution = branching_scheme.to_solution(output.solution_pool.best());

    EXPECT_TRUE(solution.full());
    // First bin's waste: (150 * 100) - (100 * 100) = 5000. Cost: 2 * 5000 = 10000.
    EXPECT_EQ(solution.cutting_cost(), 10000);
}

TEST(RectangleGuillotine, CuttingCostTooManyAtBuild)
{
    // 3 stages with exact cuts take 4 cutting costs (bin + 3 cuts). The API
    // used to accept a fifth one; 'Instance::write' then produced a file the
    // readers refused.
    InstanceBuilder instance_builder;
    instance_builder.set_objective(Objective::BinPackingCuttingCost);
    instance_builder.set_number_of_stages(3);
    instance_builder.set_cut_type(CutType::Exact);
    instance_builder.set_item_type_stack_id(instance_builder.add_item_type(100, 100, false), 0);
    BinTypeId bin_type_id = instance_builder.add_bin_type(100, 100);
    instance_builder.set_bin_type_cost(bin_type_id, 1);
    instance_builder.set_bin_type_copies(bin_type_id, -1);
    for (Counter stage_id = 0; stage_id <= 4; ++stage_id)
        instance_builder.set_fixed_cutting_cost(stage_id, 5);

    EXPECT_THROW(instance_builder.build(), std::invalid_argument);
}

TEST(RectangleGuillotine, CuttingCostRoundTrip)
{
    // A valid cutting-cost instance must survive build -> write -> read.
    InstanceBuilder instance_builder;
    instance_builder.set_objective(Objective::BinPackingCuttingCost);
    instance_builder.set_number_of_stages(3);
    instance_builder.set_cut_type(CutType::Exact);
    instance_builder.set_item_type_stack_id(instance_builder.add_item_type(100, 100, false), 0);
    BinTypeId bin_type_id = instance_builder.add_bin_type(100, 100);
    instance_builder.set_bin_type_cost(bin_type_id, 1);
    instance_builder.set_bin_type_copies(bin_type_id, -1);
    instance_builder.set_fixed_cutting_cost(0, 10);
    instance_builder.set_variable_cutting_cost(0, 1);
    for (Counter stage_id = 1; stage_id <= 3; ++stage_id) {
        instance_builder.set_fixed_cutting_cost(stage_id, 5);
        instance_builder.set_variable_cutting_cost(stage_id, 1);
    }
    Instance instance = instance_builder.build();

    const std::string path = ::testing::TempDir()
        + "packingsolver_rectangleguillotine_cutting_cost_roundtrip.json";
    instance.write(path, InstanceFormat::Json);
    InstanceBuilder read_builder;
    std::ifstream stream(path);
    ASSERT_TRUE(stream.good());
    read_builder.read(stream);
    const Instance read_instance = read_builder.build();
    std::remove(path.c_str());

    EXPECT_EQ(read_instance.objective(), instance.objective());
    EXPECT_EQ(read_instance.parameters().number_of_stages, instance.parameters().number_of_stages);
    EXPECT_EQ(read_instance.parameters().cut_type, instance.parameters().cut_type);
    ASSERT_EQ(
            read_instance.parameters().cutting_costs.size(),
            instance.parameters().cutting_costs.size());
    for (Counter stage_id = 0; stage_id < (Counter)instance.parameters().cutting_costs.size(); ++stage_id) {
        EXPECT_EQ(
                read_instance.parameters().cutting_costs[stage_id].fixed,
                instance.parameters().cutting_costs[stage_id].fixed);
        EXPECT_EQ(
                read_instance.parameters().cutting_costs[stage_id].variable,
                instance.parameters().cutting_costs[stage_id].variable);
    }
}
