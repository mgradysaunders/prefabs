$(document).ready(function() {
    $("a").each(function() {
        var text = $(this).text();
        $(this).text(text.replace("operator &", "operator&"));
    });
    $("td.memTemplParams, .memtemplate").each(function() {
        var text = $(this).text();
        text = text.replace(/template\</g, "template <")
        text = text.replace(/([a-zA-Z0-9_:]+)[ ]+([,>])/g, "$1$2")
        $(this).text(text);
    });
    $("td.memItemRight, td.memTemplItemRight").each(function() {
        var nodes = [];
        for (var index = 0; index < this.childNodes.length; index++) {
            var child = this.childNodes[index];
            if (child.nodeType == 3) {
                nodes.push(child);
            }
        }
        for (var index = 0; index < nodes.length; index++) {
            nodes[index].parentNode.replaceChild(
                document.createTextNode(nodes[index].nodeValue
                    .replace(/[ ]+([&*])([a-zA-Z_])/g, "$1 $2")
                    .replace(/-\>/, "→")
                    .replace(/\<[ ]+/g, "<")
                    .replace(/[ ]+\>/g, ">")), nodes[index]);
        }
    });
    $("td.memTemplItemLeft, td.memname").each(function() {
        var nodes = [];
        for (var index = 0; index < this.childNodes.length; index++) {
            var child = this.childNodes[index];
            if (child.nodeType == 3) {
                nodes.push(child);
            }
        }
        for (var index = 0; index < nodes.length; index++) {
            nodes[index].parentNode.replaceChild(
                document.createTextNode(nodes[index].nodeValue
                    .replace(/std::enable_if_t/, "ENABLE_IF")
                    .replace(/std::common_type_t/g, "COMMON")
                    .replace(/std::is_floating_point\<(.+?)\>::value/g, "FLOATING<$1>")
                    .replace(/std::is_integral\<(.+?)\>::value/g, "INTEGRAL<$1>")
                    .replace(/std::is_unsigned\<(.+?)\>::value/g, "UNSIGNED<$1>")
                    .replace(/std::is_signed\<(.+?)\>::value/g, "SIGNED<$1>")
                    .replace(/std::is_arithmetic\<(.+?)\>::value/g, "ARITHMETIC<$1>")
                    .replace(/is_multi\<(.+?)\>::value/g, "MULTI<$1>")
                    .replace(/is_dualnum_param\<(.+?)\>::value/g, "DUALNUM_PARAM<$1>")
                    .replace(/is_quat_param\<(.+?)\>::value/g, "QUAT_PARAM<$1>")
                    .replace(/([a-zA-Z_>])[|][|]/g, "$1 ||")
                    .replace(/([a-zA-Z_>])[&][&]/g, "$1 &&")
                    .replace(/[|][|]([a-zA-Z_])/g, "|| $1")
                    .replace(/[&][&]([a-zA-Z_])/g, "&& $1")),
                    nodes[index]);
        }
    });
    $(".classindex").wrap("<div style='overflow-x:auto'></div>");
});
