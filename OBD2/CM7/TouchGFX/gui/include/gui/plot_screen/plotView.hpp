#ifndef PLOTVIEW_HPP
#define PLOTVIEW_HPP

#include <gui_generated/plot_screen/plotViewBase.hpp>
#include <gui/plot_screen/plotPresenter.hpp>

class plotView : public plotViewBase
{
public:
    plotView();
    virtual ~plotView() {}
    virtual void setupScreen();
    virtual void tearDownScreen();
    virtual void plot_update_exe();

    void dynamicGraph_ClickHandler(const GraphWrapAndOverwrite<100>& b, const ClickEvent& e);

protected:
    Callback<plotView, const GraphWrapAndOverwrite<100>&, const ClickEvent&> plotview_voltage_callback;
    Callback<plotView, const GraphWrapAndOverwrite<100>&, const ClickEvent&> plotview_current_callback;
};

#endif // PLOTVIEW_HPP
