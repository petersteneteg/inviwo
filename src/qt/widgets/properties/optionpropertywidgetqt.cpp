#include <inviwo/qt/widgets/properties/optionpropertywidgetqt.h>
#include <QComboBox>
#include <typeinfo>
namespace inviwo {
OptionPropertyWidgetQt::OptionPropertyWidgetQt(BaseOptionProperty* property) : property_(property) { 
    generateWidget();
    updateFromProperty();
}

void OptionPropertyWidgetQt::generateWidget() {
    QHBoxLayout* hLayout = new QHBoxLayout();
    comboBox_ = new QComboBox();
    fillComboBox();
    updateFromProperty();
    hLayout->addWidget(new QLabel(QString::fromStdString(property_->getDisplayName())));
    hLayout->addWidget(comboBox_);
    setLayout(hLayout);
    connect(comboBox_, SIGNAL(currentIndexChanged(int)),this, SLOT(optionChanged()));
}

void OptionPropertyWidgetQt::fillComboBox() {
    size_t size = property_->getOptionKeys().size();
    for (size_t i=0; i < size; i++) {
        comboBox_->addItem(QString::fromStdString(property_->getOptionKeys().at(i)));
    }
}
void OptionPropertyWidgetQt::optionChanged() {
    property_->setSelectedOption(comboBox_->currentText().toLocal8Bit().constData());
}

void OptionPropertyWidgetQt::updateFromProperty() {
    int index = property_->getSelectedOption();
    comboBox_->setCurrentIndex(index);
}


} // namespace
