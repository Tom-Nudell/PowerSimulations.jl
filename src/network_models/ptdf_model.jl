function ptdf_networkflow(
    optimization_container::OptimizationContainer,
    branches::IS.FlattenIteratorWrapper{B},
    buses::IS.FlattenIteratorWrapper{PSY.Bus},
    expression::Symbol,
    PTDF::PSY.PTDF,
) where {B <: PSY.Branch}
    time_steps = model_time_steps(optimization_container)
    network_flow =
        add_cons_container!(optimization_container, :network_flow, PTDF.axes[1], time_steps)

    constraint_val = JuMPConstraintArray(undef, time_steps)
    assign_constraint!(optimization_container, "CopperPlateBalance", constraint_val)
    nodal_balance_expressions = optimization_container.expressions[expression]

    branch_types = typeof.(branches)

    remove_undef!(nodal_balance_expressions)

    for btype in Set(branch_types)
        typed_branches = IS.FlattenIteratorWrapper(
            btype,
            Vector([[b for b in branches if isa(b, btype)]]),
        )
        add_variables!(optimization_container, StandardPTDFModel(), typed_branches)
    end
    bus_count = length(buses)
    TimerOutputs.@timeit BUILD_PROBLEMS_TIMER "Branch Flows" begin
    for t in time_steps
        @show t
        @show  "Total operation count $(optimization_container.JuMPmodel.operator_counter)"
        for br in branches
            flow_variable =
                get_variable(optimization_container, FLOW_ACTIVE_POWER, typeof(br))
            name = get_name(br)
            #line_flow =
            #    model_has_parameters(optimization_container) ? zero(PGAE) :
            #    JuMP.AffExpr(0.0)
            #for b in buses
            #    bus_number = PSY.get_number(b)
            #    JuMP.add_to_expression!(line_flow, PTDF[name, bus_number] * nodal_balance_expressions[bus_number, t])
            #end
            network_flow[name, t] = JuMP.@constraint(
                optimization_container.JuMPmodel,
                flow_variable[name, t] == sum(
                 PTDF[name, PSY.get_number(b)] *
                 nodal_balance_expressions[PSY.get_number(b), t] for b in buses
             )
            )
        end
    end
           @show "finished flows"
        # The process is done in two separate loops to avoid modifying the nodal_balance_expressions
        # before making the flow constraints. If this two operations are done in the same loop
        # then the PTDF will multiply an expression that contains the flow variable.
        #for br in branches
        #    name = PSY.get_name(br)
        #    from_number = PSY.get_number(PSY.get_arc(br).from)
        #    to_number = PSY.get_number(PSY.get_arc(br).to)
        #    flow_variable =
        #        get_variable(optimization_container, FLOW_ACTIVE_POWER, typeof(br))
        #    add_to_expression!(
        #        nodal_balance_expressions,
        #        from_number,
        #        t,
        #        flow_variable[name, t],
        #        -1.0,
        #    )
        #    add_to_expression!(
        #        nodal_balance_expressions,
        #        to_number,
        #        t,
        #        flow_variable[name, t],
        #        1.0,
        #    )
        #end
        constraint_val[t] = JuMP.@constraint(
            optimization_container.JuMPmodel,
            sum(nodal_balance_expressions.data[i, t] for i in 1:bus_count) == 0
        )
         @show  "Total operation count $(optimization_container.JuMPmodel.operator_counter)"
         @show "made system balance"
    end
    return
end
