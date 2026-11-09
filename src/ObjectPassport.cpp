#include "passive_flight/ObjectPassport.hpp"

#include "passive_flight/KnownObjects.hpp"

#include <algorithm>
#include <cmath>
#include <string>
#include <unordered_set>
#include <utility>
#include <vector>

namespace passive_flight {
    namespace {
        /// Абсолютный допуск сравнения паспортных чисел с моделью;
        /// применяется в единицах сравниваемого параметра.
        constexpr double kComparisonTolerance = 1.0e-12;

        /// Добавляет нарушение, если физический параметр не конечен
        /// или не является строго положительным.
        /// @param issues Накопитель диагностических записей.
        /// @param value Проверяемое значение в единицах параметра.
        /// @param field Стабильный путь параметра в паспорте.
        void requirePositive(
            ValidationIssues &issues,
            double value,
            const std::string &field
        ) {
            if (!std::isfinite(value) || value <= 0.0) {
                issues.push_back({
                    field,
                    "Value must be finite and greater than zero"
                });
            }
        }

        /// Добавляет нарушение, если числовой параметр не конечен.
        /// @param issues Накопитель диагностических записей.
        /// @param value Проверяемое значение в единицах параметра.
        /// @param field Стабильный путь параметра в паспорте.
        void requireFinite(
            ValidationIssues &issues,
            double value,
            const std::string &field
        ) {
            if (!std::isfinite(value)) {
                issues.push_back({field, "Value must be finite"});
            }
        }

        /// Сравнивает два паспортных числа с фиксированным абсолютным допуском.
        /// @param first Первое значение в единицах параметра.
        /// @param second Второе значение в тех же единицах.
        /// @return `true`, если `abs(first-second) <= kComparisonTolerance`.
        bool valuesEqual(double first, double second) {
            return std::abs(first - second) <= kComparisonTolerance;
        }

        /// Добавляет числовую запись о происхождении параметра.
        /// @param passport Изменяемый паспорт объекта.
        /// @param path Стабильный путь параметра.
        /// @param value Числовое значение в единице, заданной параметром unit.
        /// @param unit Обозначение единицы СИ либо `1` для безразмерного числа.
        /// @param status Категория способа получения значения.
        /// @param source Идентификатор источника или расчётного правила.
        /// @param note Пояснение физического смысла параметра.
        void addNumber(
            ObjectPassport &passport,
            std::string path,
            double value,
            std::string unit,
            ParameterStatus status,
            std::string source,
            std::string note
        ) {
            passport.parameters.push_back({
                std::move(path),
                value,
                std::move(unit),
                status,
                std::move(source),
                std::move(note)
            });
        }

        /// Добавляет текстовую запись о происхождении параметра.
        /// @param passport Изменяемый паспорт объекта.
        /// @param path Стабильный путь параметра.
        /// @param value Текстовое значение.
        /// @param status Категория способа получения значения.
        /// @param source Идентификатор источника или архитектурного решения.
        /// @param note Пояснение смысла записи.
        void addText(
            ObjectPassport &passport,
            std::string path,
            std::string value,
            ParameterStatus status,
            std::string source,
            std::string note
        ) {
            passport.parameters.push_back({
                std::move(path),
                std::move(value),
                "-",
                status,
                std::move(source),
                std::move(note)
            });
        }

        /// Проверяет наличие числовой записи и её совпадение с моделью.
        /// @param issues Накопитель нарушений паспорта.
        /// @param passport Проверяемый паспорт.
        /// @param path Путь искомой записи.
        /// @param expectedValue Ожидаемое значение в единицах записи.
        void checkNumericRecord(
            ValidationIssues &issues,
            const ObjectPassport &passport,
            const std::string &path,
            double expectedValue
        ) {
            const ParameterRecord *record = findParameter(passport, path);

            if (record == nullptr) {
                issues.push_back({path, "Parameter provenance record is missing"});
                return;
            }

            const double *recordedValue = std::get_if<double>(&record->value);

            if (recordedValue == nullptr) {
                issues.push_back({
                    path,
                    "Parameter provenance record must contain a number"
                });
                return;
            }

            if (!valuesEqual(*recordedValue, expectedValue)) {
                issues.push_back({
                    path,
                    "Parameter record does not match the object model"
                });
            }
        }

        /// Проверяет наличие текстовой записи и её совпадение с моделью.
        /// @param issues Накопитель нарушений паспорта.
        /// @param passport Проверяемый паспорт.
        /// @param path Путь искомой записи.
        /// @param expectedValue Ожидаемая текстовая величина.
        void checkTextRecord(
            ValidationIssues &issues,
            const ObjectPassport &passport,
            const std::string &path,
            const std::string &expectedValue
        ) {
            const ParameterRecord *record = findParameter(passport, path);

            if (record == nullptr) {
                issues.push_back({path, "Parameter provenance record is missing"});
                return;
            }

            const std::string *recordedValue =
                    std::get_if<std::string>(&record->value);

            if (recordedValue == nullptr) {
                issues.push_back({
                    path,
                    "Parameter provenance record must contain text"
                });
                return;
            }

            if (*recordedValue != expectedValue) {
                issues.push_back({
                    path,
                    "Parameter record does not match the object model"
                });
            }
        }

        /// Проверяет структуру одной таблицы зависимости от числа Маха.
        /// @param issues Накопитель нарушений числовой модели.
        /// @param table Таблица безразмерных чисел Маха и коэффициентов.
        /// @param field Путь таблицы в ObjectModel.
        void validateMachTable(
            ValidationIssues &issues,
            const std::vector<MachCoefficientPoint> &table,
            const std::string &field
        ) {
            if (table.size() < 2) {
                issues.push_back({
                    field,
                    "Aerodynamic table must contain at least two points"
                });
                return;
            }

            double previousMach = 0.0;
            bool first = true;

            for (const MachCoefficientPoint &point: table) {
                if (!std::isfinite(point.mach) || point.mach < 0.0) {
                    issues.push_back({
                        field,
                        "Mach values must be finite and non-negative"
                    });
                    return;
                }

                if (!std::isfinite(point.value)) {
                    issues.push_back({field, "Coefficient values must be finite"});
                    return;
                }

                if (!first && point.mach <= previousMach) {
                    issues.push_back({
                        field,
                        "Mach values must be strictly increasing"
                    });
                    return;
                }

                previousMach = point.mach;
                first = false;
            }
        }

        /// Проверяет идентификационные, массовые и геометрические поля объекта.
        /// @param issues Накопитель обнаруженных нарушений.
        /// @param object Модель с массой в кг, инерцией в кг·м² и геометрией в м.
        void validateObjectFields(
            ValidationIssues &issues,
            const ObjectModel &object
        ) {
            if (object.id.empty()) {
                issues.push_back({"object.id", "Object identifier must not be empty"});
            }

            if (object.metadata.displayName.empty()) {
                issues.push_back({
                    "object.metadata.displayName",
                    "Display name must not be empty"
                });
            }

            if (object.metadata.modelVersion.empty()) {
                issues.push_back({
                    "object.metadata.modelVersion",
                    "Model version must not be empty"
                });
            }

            requirePositive(issues, object.mass.massKg, "mass.massKg");
            requirePositive(
                issues,
                object.mass.pitchMomentOfInertiaKgM2,
                "mass.pitchMomentOfInertiaKgM2"
            );
            requireFinite(issues, object.mass.centerOfMassXM, "mass.centerOfMassXM");
            requirePositive(issues, object.reference.areaM2, "reference.areaM2");
            requirePositive(
                issues,
                object.reference.referenceLengthM,
                "reference.referenceLengthM"
            );
            requirePositive(issues, object.body.lengthM, "body.lengthM");
            requirePositive(issues, object.body.diameterM, "body.diameterM");

            if (std::isfinite(object.mass.centerOfMassXM) &&
                std::isfinite(object.body.lengthM) &&
                (object.mass.centerOfMassXM < 0.0 ||
                 object.mass.centerOfMassXM > object.body.lengthM)) {
                issues.push_back({
                    "mass.centerOfMassXM",
                    "Center of mass must lie inside the body length"
                });
            }

            if (object.aerodynamicModelType != AerodynamicModelType::Tabulated) {
                issues.push_back({
                    "aerodynamics.modelType",
                    "Only tabulated aerodynamics is supported"
                });
            }
        }

        /// Проверяет обязательные аэродинамические таблицы и наличие общего
        /// ненулевого интервала безразмерного числа Маха.
        /// @param issues Накопитель обнаруженных нарушений.
        /// @param data Набор табличных коэффициентов и производных.
        void validateTabulatedAerodynamics(
            ValidationIssues &issues,
            const TabulatedAerodynamicData &data
        ) {
            validateMachTable(issues, data.cx0, "tabulatedAerodynamics.cx0");
            validateMachTable(
                issues,
                data.cxAlphaSquared,
                "tabulatedAerodynamics.cxAlphaSquared"
            );
            validateMachTable(issues, data.cyAlpha, "tabulatedAerodynamics.cyAlpha");
            validateMachTable(issues, data.mzAlpha, "tabulatedAerodynamics.mzAlpha");
            validateMachTable(
                issues,
                data.mzPitchRate,
                "tabulatedAerodynamics.mzPitchRate"
            );

            if (data.cx0.empty() || data.cxAlphaSquared.empty() ||
                data.cyAlpha.empty() || data.mzAlpha.empty() ||
                data.mzPitchRate.empty()) {
                return;
            }

            const double commonMinimumMach = std::max({
                data.cx0.front().mach,
                data.cxAlphaSquared.front().mach,
                data.cyAlpha.front().mach,
                data.mzAlpha.front().mach,
                data.mzPitchRate.front().mach
            });

            const double commonMaximumMach = std::min({
                data.cx0.back().mach,
                data.cxAlphaSquared.back().mach,
                data.cyAlpha.back().mach,
                data.mzAlpha.back().mach,
                data.mzPitchRate.back().mach
            });

            if (commonMaximumMach <= commonMinimumMach) {
                issues.push_back({
                    "tabulatedAerodynamics",
                    "Aerodynamic tables must have a common Mach interval"
                });
            }
        }

        /// Заполняет паспорт ФАБ-1500Т записями происхождения параметров
        /// из таблицы 1.1 Постникова—Чуйко и производных расчётных величин.
        /// @param passport Паспорт с предварительно созданной числовой моделью.
        void appendFab1500TProvenance(ObjectPassport &passport) {
            const ObjectModel &object = passport.object;
            constexpr const char *source = "postnikov-chuyko-1979-table-1.1";

            addNumber(passport, "mass.massKg", object.mass.massKg, "kg",
                      ParameterStatus::SourceDocument, source, "Масса ФАБ-1500Т");
            addNumber(passport, "mass.pitchMomentOfInertiaKgM2",
                      object.mass.pitchMomentOfInertiaKgM2, "kg*m^2",
                      ParameterStatus::SourceDocument, source,
                      "Поперечный момент инерции Jzz");
            addNumber(passport, "mass.centerOfMassXM", object.mass.centerOfMassXM,
                      "m", ParameterStatus::SourceDocument, source,
                      "Координата центра масс от носа");
            addNumber(passport, "body.lengthM", object.body.lengthM, "m",
                      ParameterStatus::SourceDocument, source, "Полная длина ФАБ-1500Т");
            addNumber(passport, "body.diameterM", object.body.diameterM, "m",
                      ParameterStatus::SourceDocument, source,
                      "Максимальный диаметр ФАБ-1500Т");
            addNumber(passport, "reference.areaM2", object.reference.areaM2, "m^2",
                      ParameterStatus::Derived, source, "Площадь миделя pi*D^2/4");
            addNumber(passport, "reference.referenceLengthM",
                      object.reference.referenceLengthM, "m",
                      ParameterStatus::SourceDocument, source,
                      "Характерная длина коэффициента момента равна длине бомбы");
            addText(passport, "aerodynamics.modelType",
                    aerodynamicModelTypeName(object.aerodynamicModelType),
                    ParameterStatus::Derived, "model-architecture",
                    "Готовые табличные аэродинамические характеристики");
            addText(passport, "aerodynamics.source", source,
                    ParameterStatus::SourceDocument, source,
                    "Источник пяти аэродинамических таблиц ФАБ-1500Т");
            addNumber(passport, "aerodynamics.cx0.pointCount",
                      static_cast<double>(object.tabulatedAerodynamics.cx0.size()), "1",
                      ParameterStatus::Derived, source, "Количество узлов Cx0(M)");
            addNumber(passport, "aerodynamics.cxAlphaSquared.pointCount",
                      static_cast<double>(object.tabulatedAerodynamics.cxAlphaSquared.size()),
                      "1", ParameterStatus::Derived, source,
                      "Количество узлов Cx^(alpha^2)(M)");
            addNumber(passport, "aerodynamics.cyAlpha.pointCount",
                      static_cast<double>(object.tabulatedAerodynamics.cyAlpha.size()), "1",
                      ParameterStatus::Derived, source, "Количество узлов Cy^alpha(M)");
            addNumber(passport, "aerodynamics.mzAlpha.pointCount",
                      static_cast<double>(object.tabulatedAerodynamics.mzAlpha.size()), "1",
                      ParameterStatus::Derived, source, "Количество узлов mz^alpha(M)");
            addNumber(passport, "aerodynamics.mzPitchRate.pointCount",
                      static_cast<double>(object.tabulatedAerodynamics.mzPitchRate.size()),
                      "1", ParameterStatus::Derived, source,
                      "Количество узлов mz^(omega_bar)(M)");
        }

        /// Проверяет уникальность путей, наличие единиц и источников,
        /// а также конечность числовых значений паспортных записей.
        /// @param issues Накопитель нарушений паспорта.
        /// @param passport Проверяемая последовательность записей.
        void validateParameterList(
            ValidationIssues &issues,
            const ObjectPassport &passport
        ) {
            std::unordered_set<std::string> uniquePaths;

            for (const ParameterRecord &record: passport.parameters) {
                if (record.path.empty()) {
                    issues.push_back({"parameters", "Parameter path must not be empty"});
                }
                if (!uniquePaths.insert(record.path).second) {
                    issues.push_back({
                        record.path,
                        "Duplicate parameter provenance record"
                    });
                }
                if (record.unit.empty()) {
                    issues.push_back({record.path, "Parameter unit must not be empty"});
                }
                if (record.source.empty()) {
                    issues.push_back({record.path, "Parameter source must not be empty"});
                }
                if (const double *number = std::get_if<double>(&record.value);
                    number != nullptr && !std::isfinite(*number)) {
                    issues.push_back({
                        record.path,
                        "Recorded parameter value must be finite"
                    });
                }
            }
        }

        /// Сверяет ключевые записи происхождения контрольной ФАБ-1500Т
        /// с фактическими значениями числовой модели.
        /// @param issues Накопитель расхождений паспорта и модели.
        /// @param passport Контрольный паспорт по Постникову—Чуйко.
        void validateFab1500TProvenance(
            ValidationIssues &issues,
            const ObjectPassport &passport
        ) {
            const ObjectModel &object = passport.object;

            checkNumericRecord(issues, passport, "mass.massKg", object.mass.massKg);
            checkNumericRecord(issues, passport, "mass.pitchMomentOfInertiaKgM2",
                               object.mass.pitchMomentOfInertiaKgM2);
            checkNumericRecord(issues, passport, "mass.centerOfMassXM",
                               object.mass.centerOfMassXM);
            checkNumericRecord(issues, passport, "body.lengthM", object.body.lengthM);
            checkNumericRecord(issues, passport, "body.diameterM",
                               object.body.diameterM);
            checkNumericRecord(issues, passport, "reference.areaM2",
                               object.reference.areaM2);
            checkNumericRecord(issues, passport, "reference.referenceLengthM",
                               object.reference.referenceLengthM);
            checkTextRecord(issues, passport, "aerodynamics.modelType", "tabulated");
            checkTextRecord(issues, passport, "aerodynamics.source",
                            "postnikov-chuyko-1979-table-1.1");
            checkNumericRecord(issues, passport, "aerodynamics.cx0.pointCount",
                               static_cast<double>(object.tabulatedAerodynamics.cx0.size()));
            checkNumericRecord(issues, passport,
                               "aerodynamics.cxAlphaSquared.pointCount",
                               static_cast<double>(object.tabulatedAerodynamics.cxAlphaSquared.size()));
            checkNumericRecord(issues, passport, "aerodynamics.cyAlpha.pointCount",
                               static_cast<double>(object.tabulatedAerodynamics.cyAlpha.size()));
            checkNumericRecord(issues, passport, "aerodynamics.mzAlpha.pointCount",
                               static_cast<double>(object.tabulatedAerodynamics.mzAlpha.size()));
            checkNumericRecord(issues, passport, "aerodynamics.mzPitchRate.pointCount",
                               static_cast<double>(object.tabulatedAerodynamics.mzPitchRate.size()));
        }
    } // namespace

    ObjectPassport makeFab1500TPostnikovPassport() {
        ObjectPassport passport;
        passport.object = makeFab1500TPostnikovModel();
        appendFab1500TProvenance(passport);
        return passport;
    }

    ObjectPassport makeFab1500TModifiedPassport() {
        ObjectPassport passport;
        passport.object = makeFab1500TModifiedModel();

        const ObjectModel &object = passport.object;
        constexpr const char *source = "user-provided-table-interpolated-cx0-plus-5pct";

        addNumber(passport, "mass.massKg", object.mass.massKg, "kg",
                  ParameterStatus::SourceDocument, source, "Масса ФАБ-1500Т");
        addNumber(passport, "mass.pitchMomentOfInertiaKgM2",
                  object.mass.pitchMomentOfInertiaKgM2, "kg*m^2",
                  ParameterStatus::SourceDocument, source,
                  "Поперечный момент инерции Jzz");
        addNumber(passport, "mass.centerOfMassXM", object.mass.centerOfMassXM,
                  "m", ParameterStatus::SourceDocument, source,
                  "Координата центра масс от носа");
        addNumber(passport, "body.lengthM", object.body.lengthM, "m",
                  ParameterStatus::SourceDocument, source, "Полная длина ФАБ-1500Т");
        addNumber(passport, "body.diameterM", object.body.diameterM, "m",
                  ParameterStatus::SourceDocument, source,
                  "Максимальный диаметр ФАБ-1500Т");
        addNumber(passport, "reference.areaM2", object.reference.areaM2, "m^2",
                  ParameterStatus::Derived, source, "Площадь миделя pi*D^2/4");
        addNumber(passport, "reference.referenceLengthM",
                  object.reference.referenceLengthM, "m",
                  ParameterStatus::SourceDocument, source,
                  "Характерная длина коэффициента момента равна длине бомбы");
        addText(passport, "aerodynamics.modelType",
                aerodynamicModelTypeName(object.aerodynamicModelType),
                ParameterStatus::Derived, "model-architecture",
                "Готовые табличные аэродинамические характеристики");
        addText(passport, "aerodynamics.source", source,
                ParameterStatus::SourceDocument, source,
                "Источник: предоставленная таблица Cy/mz + Cx0*1.05");
        addNumber(passport, "aerodynamics.cx0.pointCount",
                  static_cast<double>(object.tabulatedAerodynamics.cx0.size()), "1",
                  ParameterStatus::Derived, source, "Количество узлов Cx0(M)");
        addNumber(passport, "aerodynamics.cxAlphaSquared.pointCount",
                  static_cast<double>(object.tabulatedAerodynamics.cxAlphaSquared.size()),
                  "1", ParameterStatus::Derived, source,
                  "Количество узлов Cx^(alpha^2)(M)");
        addNumber(passport, "aerodynamics.cyAlpha.pointCount",
                  static_cast<double>(object.tabulatedAerodynamics.cyAlpha.size()), "1",
                  ParameterStatus::Derived, source, "Количество узлов Cy^alpha(M)");
        addNumber(passport, "aerodynamics.mzAlpha.pointCount",
                  static_cast<double>(object.tabulatedAerodynamics.mzAlpha.size()), "1",
                  ParameterStatus::Derived, source, "Количество узлов mz^alpha(M)");
        addNumber(passport, "aerodynamics.mzPitchRate.pointCount",
                  static_cast<double>(object.tabulatedAerodynamics.mzPitchRate.size()),
                  "1", ParameterStatus::Derived, source,
                  "Количество узлов mz^(omega_bar)(M)");

        return passport;
    }

    const ParameterRecord *findParameter(
        const ObjectPassport &passport,
        const std::string &path
    ) noexcept {
        const auto iterator = std::find_if(
            passport.parameters.begin(),
            passport.parameters.end(),
            [&path](const ParameterRecord &record) {
                return record.path == path;
            }
        );

        return iterator == passport.parameters.end() ? nullptr : &(*iterator);
    }

    ValidationIssues validate(const ObjectModel &object) {
        ValidationIssues issues;
        validateObjectFields(issues, object);
        validateTabulatedAerodynamics(issues, object.tabulatedAerodynamics);
        return issues;
    }

    ValidationIssues validate(const ObjectPassport &passport) {
        ValidationIssues issues = validate(passport.object);
        validateParameterList(issues, passport);

        /*
         * Полная сверка с первичным источником выполняется только для
         * контрольной модели; расчётные модификации имеют иной статус данных.
         */
        if (passport.object.id == "FAB_1500T_POSTNIKOV_1979") {
            validateFab1500TProvenance(issues, passport);
        }

        return issues;
    }

    const char *parameterStatusName(ParameterStatus status) noexcept {
        switch (status) {
            case ParameterStatus::Requirement:
                return "requirement";
            case ParameterStatus::SourceDocument:
                return "source_document";
            case ParameterStatus::Derived:
                return "derived";
            case ParameterStatus::Provisional:
                return "provisional";
            case ParameterStatus::AuxiliaryTable:
                return "auxiliary_table";
        }

        return "unknown";
    }

    const char *aerodynamicModelTypeName(AerodynamicModelType type) noexcept {
        switch (type) {
            case AerodynamicModelType::Tabulated:
                return "tabulated";
        }

        return "unknown";
    }
     ObjectPassport makeFab1500TGlidePassport() {
        ObjectPassport passport;
        passport.object = makeFab1500TGlideModel();

        const ObjectModel &object = passport.object;
        constexpr const char *source = "lab-work-scaled-plus-trim-1deg";

        addNumber(passport, "mass.massKg", object.mass.massKg, "kg",
                  ParameterStatus::SourceDocument, source, "Масса ФАБ-1500Т");
        addNumber(passport, "mass.pitchMomentOfInertiaKgM2",
                  object.mass.pitchMomentOfInertiaKgM2, "kg*m^2",
                  ParameterStatus::SourceDocument, source,
                  "Поперечный момент инерции Jzz");
        addNumber(passport, "mass.centerOfMassXM", object.mass.centerOfMassXM,
                  "m", ParameterStatus::SourceDocument, source,
                  "Координата центра масс от носа");
        addNumber(passport, "body.lengthM", object.body.lengthM, "m",
                  ParameterStatus::SourceDocument, source, "Полная длина");
        addNumber(passport, "body.diameterM", object.body.diameterM, "m",
                  ParameterStatus::SourceDocument, source, "Диаметр миделя");
        addNumber(passport, "reference.areaM2", object.reference.areaM2, "m^2",
                  ParameterStatus::Derived, source, "Площадь миделя");
        addNumber(passport, "reference.referenceLengthM",
                  object.reference.referenceLengthM, "m",
                  ParameterStatus::SourceDocument, source,
                  "Характерная длина = длина корпуса");
        addText(passport, "aerodynamics.modelType",
                aerodynamicModelTypeName(object.aerodynamicModelType),
                ParameterStatus::Derived, "model-architecture",
                "Табличная модель с Cy0 и mz0");
        addText(passport, "aerodynamics.source", source,
                ParameterStatus::SourceDocument, source,
                "Масштабированные производные + триммирование 1°");
        addNumber(passport, "aerodynamics.cy0.pointCount",
                  static_cast<double>(object.tabulatedAerodynamics.cy0.size()), "1",
                  ParameterStatus::Derived, source, "Количество узлов Cy0(M)");
        addNumber(passport, "aerodynamics.mz0.pointCount",
                  static_cast<double>(object.tabulatedAerodynamics.mz0.size()), "1",
                  ParameterStatus::Derived, source, "Количество узлов mz0(M)");

        return passport;
    }
} // namespace passive_flight


