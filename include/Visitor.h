#ifndef VISITOR_H
#define VISITOR_H

namespace ngc {
    class VisitorContext;

    class Visitor {
    public:
        virtual ~Visitor() = default;

        // statements
        virtual void visit(const class CompoundStatement *stmt, VisitorContext *ctx) = 0;
        virtual void visit(const class BlockStatement *stmt, VisitorContext *ctx) = 0;
        virtual void visit(const class SubStatement *stmt, VisitorContext *ctx) = 0;
        virtual void visit(const class IfStatement *stmt, VisitorContext *ctx) = 0;
        virtual void visit(const class WhileStatement *stmt, VisitorContext *ctx) = 0;
        virtual void visit(const class ReturnStatement *stmt, VisitorContext *ctx) = 0;
        virtual void visit(const class AliasStatement *stmt, VisitorContext *ctx) = 0;
        virtual void visit(const class LetStatement *stmt, VisitorContext *ctx) = 0;

        // expressions
        virtual void visit(const class CommentExpression *expr, VisitorContext *ctx) = 0;
        virtual void visit(const class WordExpression *expr, VisitorContext *ctx) = 0;
        virtual void visit(const class LiteralExpression *expr, VisitorContext *ctx) = 0;
        virtual void visit(const class StringExpression *expr, VisitorContext *ctx) = 0;
        virtual void visit(const class NumericVariableExpression *expr, VisitorContext *ctx) = 0;
        virtual void visit(const class NamedVariableExpression *expr, VisitorContext *ctx) = 0;
        virtual void visit(const class UnaryExpression *expr, VisitorContext *ctx) = 0;
        virtual void visit(const class BinaryExpression *expr, VisitorContext *ctx) = 0;
        virtual void visit(const class CallExpression *expr, VisitorContext *ctx) = 0;
        virtual void visit(const class GroupingExpression *expr, VisitorContext *ctx) = 0;
    };
}

#endif //VISITOR_H
