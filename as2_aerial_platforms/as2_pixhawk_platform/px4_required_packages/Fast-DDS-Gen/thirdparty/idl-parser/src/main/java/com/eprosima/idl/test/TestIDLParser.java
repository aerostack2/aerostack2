package com.eprosima.idl.test;


import java.io.BufferedReader;
import java.io.FileNotFoundException;
import java.io.FileOutputStream;
import java.io.IOException;
import java.io.InputStream;
import java.io.InputStreamReader;
import java.io.OutputStream;
import java.lang.reflect.AnnotatedElement;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.Collection;

import org.antlr.v4.runtime.ANTLRFileStream;
import org.antlr.v4.runtime.CommonTokenStream;


import com.eprosima.idl.context.Context;
import com.eprosima.idl.parser.grammar.IDLLexer;
import com.eprosima.idl.parser.grammar.IDLParser;
import com.eprosima.idl.parser.tree.Annotation;
import com.eprosima.idl.parser.tree.AnnotationDeclaration;
import com.eprosima.idl.parser.tree.AnnotationMember;
import com.eprosima.idl.parser.tree.ConstDeclaration;
import com.eprosima.idl.parser.tree.Definition;
import com.eprosima.idl.parser.tree.Interface;
import com.eprosima.idl.parser.tree.Module;
import com.eprosima.idl.parser.tree.Specification;
import com.eprosima.idl.parser.tree.TypeDeclaration;
import com.eprosima.idl.parser.typecode.*;
import com.eprosima.idl.util.Util;
import com.eprosima.log.ColorMessage;


public class TestIDLParser {

    private String m_tempDir = null;
    private String m_os = null;
    private String m_ppPath = null;
    private ArrayList<String> m_includePaths = new ArrayList<String>();

    public TestIDLParser() {
		 m_os = System.getProperty("os.name");
    }

    public void addIncludePath(String path)
    {
        m_includePaths.add("-I" + path);
    }

    public void setPreprocessorPath(String path)
    {
        m_ppPath = path;
    }

    public void setTemporalPath(String path)
    {
        m_tempDir = path;
    }

    public void parse(String idlFileName) {

	    System.out.println("Start Parsing IDL File: " + idlFileName + "\n");

		//m_includePaths.add("-Ie:\\data\\idl");

	    String onlyFileName = Util.getIDLFileNameOnly(idlFileName);

	    String idlParseFileName = callPreprocessor(idlFileName);

	    if (idlParseFileName == null)
	    {
	        System.out.println("Error calling preprocessor.");
	        return;
	    }

	    Context context = new Context(onlyFileName, idlFileName, m_includePaths);

	    try {

            ANTLRFileStream input = new ANTLRFileStream(idlParseFileName);
		    IDLLexer lexer = new IDLLexer(input);
		    lexer.setContext(context);
		    CommonTokenStream tokenStream = new CommonTokenStream(lexer);
		    IDLParser parser = new IDLParser(tokenStream);
            Specification specification  = parser.specification(context, null, null).spec;

            if (specification != null)
            {
		        for( Definition definition: specification.getDefinitions()) {

                    if (definition.isIsModule())
                    {
		         	    parseModule((Module)definition);
                    }
                    else if (definition.isIsInterface())
                    {
		         	    parseInterface((Interface)definition);
                    }
                    else if (definition.isIsException())
                    {
		         	    parseException((com.eprosima.idl.parser.tree.Exception)definition);
                    }
                    else if (definition.isIsTypeDeclaration())
                    {
		                parseTypeDeclaration((TypeDeclaration)definition);
                    }
                    else if (definition.isIsConstDeclaration())
                    {
		         	    parseConstDeclaration((ConstDeclaration)definition);
                    }
                    else if (definition.isIsAnnotation())
                    {
		         	    parseAnnotation((AnnotationDeclaration)definition);
                    }
                    else
                    {
		                System.out.println("Unknown Type");
		           	}
                }
            }

		} catch (IOException e) {
    		    e.printStackTrace();
    	  }
	    System.out.println("\nParsing Completed \n");

	}

    public void parseModule(Module moduleDef) {
	    System.out.println("Start Module: " + moduleDef.getName() + "\n");

        for( Definition moduleDefinition: moduleDef.getDefinitions())
        {
            if (moduleDefinition.isIsTypeDeclaration())
            {
			    parseTypeDeclaration((TypeDeclaration) moduleDefinition);
            }
            else if (moduleDefinition.isIsInterface())
            {
                parseInterface((Interface)moduleDefinition);
        	}
            else if (moduleDefinition.isIsModule())
            {
                parseModule((Module)moduleDefinition);
            }
            else if (moduleDefinition.isIsException())
            {
                parseException((com.eprosima.idl.parser.tree.Exception)moduleDefinition);
            }
            else if (moduleDefinition.isIsConstDeclaration())
            {
                parseConstDeclaration((ConstDeclaration)moduleDefinition);
            }
            else if (moduleDefinition.isIsAnnotation())
            {
                parseAnnotation((AnnotationDeclaration)moduleDefinition);
            }
            else
            {
        	    System.out.println("Module Unrecognized Option ");
        	}
		}
	    System.out.println("End Module: " + moduleDef.getName());
	}

    public void parseInterface(Interface interfaceDef) {
   	    System.out.println("Start Interface: " + interfaceDef.getName());
	    System.out.println("End Interface: \n");
	}

    public void parseException(com.eprosima.idl.parser.tree.Exception exceptionDef)
    {
        System.out.println("Start Exception: " + exceptionDef.getName());
        System.out.println("End Annotation: \n");
    }

    public void parseAnnotations(Collection<Annotation> annotations, boolean new_line, String tab)
    {
        for (Annotation annotation : annotations)
        {
            System.out.print(tab + "@" + annotation.getName());
            if (annotation.getValues().size() > 0)
            {
                boolean first = true;
                System.out.print(" (");
                for (AnnotationMember member : annotation.getValues().values())
                {
                    if (!first)
                    {
                        System.out.print(", ");
                    }
                    System.out.print(member.getName() + " = " + member.getValue());
                    first = false;
                }
                if (new_line) System.out.println(")");
                else System.out.print(") ");
            }
        }
    }

    public void parseTypeDeclaration(TypeDeclaration typeDeclarationDef)
    {
        if (typeDeclarationDef.getAnnotations().size() > 0)
        {
            parseAnnotations(typeDeclarationDef.getAnnotations().values(), true , "");
        }
	    switch (typeDeclarationDef.getTypeCode().getKind()) {
		    case Kind.KIND_STRUCT:
			    parseStruct((StructTypeCode)typeDeclarationDef.getTypeCode());
		    break;
		    case Kind.KIND_UNION:
			    parseUnion((UnionTypeCode)typeDeclarationDef.getTypeCode());
		    break;
		    case Kind.KIND_ENUM:
			    parseEnum((EnumTypeCode)typeDeclarationDef.getTypeCode());
		    break;
		    case Kind.KIND_ALIAS:
			    parseAlias((AliasTypeCode)typeDeclarationDef.getTypeCode());
		    break;
		    case Kind.KIND_BITSET:
			    parseBitset((BitsetTypeCode)typeDeclarationDef.getTypeCode());
		    break;
		    case Kind.KIND_BITMASK:
			    parseBitmask((BitmaskTypeCode)typeDeclarationDef.getTypeCode());
		    break;
		    default:
			    System.out.println("Parse Type Declaration: Not Handled ");
		}
	}

    public void parseConstDeclaration(ConstDeclaration constDeclarationDef)
    {
        System.out.println("Start ConstDeclaration: " + constDeclarationDef.getName());
        System.out.println("    - Type: " + constDeclarationDef.getTypeCode().getTypeIdentifier());
        System.out.println("    - Value: " + constDeclarationDef.getValue());
        System.out.println("End Annotation: \n");
	}

    public void parseAnnotation(AnnotationDeclaration annotationDef) {
        System.out.println("Start Annotation: " + annotationDef.getName());
        for (AnnotationMember member : annotationDef.getMembers())
        {
            System.out.println("    " + member.getTypecode().getTypeIdentifier() + " " + member.getName());
        }
        System.out.println("End Annotation: \n");
	}

    public void parseAlias(AliasTypeCode aliasType) {
	    System.out.println("Start Alias (TypeDef) ");
	    System.out.println("End Alias: \n");
    }

    public void parseMember(Member member)
    {
        if (member.getAnnotations().size() > 0)
        {
            parseAnnotations(member.getAnnotations().values(), false, "        ");
        }

        if (member.getTypecode() instanceof EnumTypeCode){
            parseEnumField(member);
        } else if (member.getTypecode() instanceof StructTypeCode){
            parseStructField(member);
        } else if (member.getTypecode() instanceof UnionTypeCode){
            parseUnionField(member);
        } else if (member.getTypecode() instanceof AliasTypeCode) {
            parseAliasField(member);
        } else if (member.getTypecode() instanceof PrimitiveTypeCode) {
            parsePrimitiveField(member);
        } else if (member.getTypecode() instanceof BitsetTypeCode) {
            parseBitsetField(member);
        } else if (member.getTypecode() instanceof BitmaskTypeCode) {
            parseBitmaskField(member);
        } else {
            parseDefaultField(member);
        }
    }

    public void parseUnionMember(UnionMember member)
    {
        for (String label : member.getLabels())
        {
            System.out.print("        " + label + " - ");
        }
        if (member.isDefault())
        {
            System.out.print("        Default - ");
        }

        if (member.getAnnotations().size() > 0)
        {
            parseAnnotations(member.getAnnotations().values(), false, "        ");
        }

        if (member.getTypecode() instanceof EnumTypeCode){
            parseEnumField(member);
        } else if (member.getTypecode() instanceof StructTypeCode){
            parseStructField(member);
        } else if (member.getTypecode() instanceof UnionTypeCode){
            parseUnionField(member);
        } else if (member.getTypecode() instanceof AliasTypeCode) {
            parseAliasField(member);
        } else if (member.getTypecode() instanceof PrimitiveTypeCode) {
            parsePrimitiveField(member);
        } else if (member.getTypecode() instanceof BitsetTypeCode) {
           parseBitsetField(member);
        } else if (member.getTypecode() instanceof BitmaskTypeCode) {
           parseBitmaskField(member);
        } else {
            parseDefaultField(member);
        }
    }

    public void parseStruct(StructTypeCode struct) {
	    System.out.println("Start Struct: " + struct.getName());
         for (Member member: struct.getMembers(true)) {
             parseMember(member);
     	}
	    System.out.println("End Struct: \n");
	}

    public void parseUnion(UnionTypeCode union) {
	    System.out.println("Start Union: " + union.getName() + " (" + union.getDiscriminator().getTypeIdentifier() + ")");
         for (Member member: union.getMembers()) {
             parseUnionMember((UnionMember)member);
     	}
	    System.out.println("End Union: \n");
	}

    public void parseBitset(BitsetTypeCode bitset) {
	    System.out.println("Start Bitset: " + bitset.getName());
         for (Bitfield field : bitset.getBitfields(true)) {
            parseBitfield(field);
     	}
	    System.out.println("End Bitset: \n");
	}

    public void parseBitmask(BitmaskTypeCode bitmask) {
	    System.out.println("Start Bitmask: " + bitmask.getName() + " (" + bitmask.getBitBound() + ")");
         for (Bitmask mask : bitmask.getBitmasks()) {
            parseBitmask(mask);
     	}
	    System.out.println("End Bitset: \n");
	}

    public static void parseBitfield(Bitfield field) {
	    System.out.println("    Bitfield: "  +  field.getName() + " (" + field.getSpec().getBitSize() + ")" );
	}

    public static void parseBitmask(Bitmask mask) {
	    System.out.println("    Bitmask: "  +  mask.getName() + " (" + mask.getPosition() + ")" );
	}

    public static void parsePrimitiveField(Member member) {
	    System.out.println("    Field " + member.getTypecode().getTypeIdentifier() + ": " + member.getName() );
	}

    public static void parseDefaultField(Member member) {
	    System.out.println("    Field " + member.getTypecode().getStType() + ": "  +  member.getName() );
	}

    public static void parseBitsetField(Member member) {
	    System.out.println("    Field bitset: "  +  member.getName() );
	}

    public static void parseBitmaskField(Member member) {
	    System.out.println("    Field bitmask: "  +  member.getName() );
	}

    public void parseEnum(EnumTypeCode enumType) {
        System.out.println("Enum: " + enumType.getName());
        for (Member member: enumType.getMembers()) {
             System.out.println("    Enum Member: " + member.getName());
    	}
        System.out.println("End Enum: \n");
	}

	// Fields Parse
    public void parseEnumField(Member member) {
	    EnumTypeCode typeCode = (EnumTypeCode) member.getTypecode();
	    System.out.println("    Field Enum: " + typeCode.getName() + " " + member.getName());
	}

    public void parseStructField(Member member) {
	    StructTypeCode typeCode = (StructTypeCode) member.getTypecode();
	    System.out.println("    Field Struct: " + typeCode.getName() + " " + member.getName());
	}

    public void parseUnionField(Member member) {
	    UnionTypeCode typeCode = (UnionTypeCode) member.getTypecode();
	    System.out.println("    Field Union: " + typeCode.getName() + " " + member.getName());
	}

    public void parseAliasField(Member member) {
	    AliasTypeCode typeCode = (AliasTypeCode)member.getTypecode();
        System.out.println("    Field Alias: " + typeCode.getName() + " " +  member.getName() );
	}


    String callPreprocessor(String idlFilename)
    {
        final String METHOD_NAME = "callPreprocessor";

        // Set line command.
        ArrayList<String> lineCommand = new ArrayList<String>();
        String [] lineCommandArray = null;
        String outputfile = Util.getIDLFileOnly(idlFilename) + ".cc";
        int exitVal = -1;
        OutputStream of = null;

        // Use temp directory.
        if (m_tempDir != null) {
            outputfile = m_tempDir + outputfile;
        }

        if (m_os.contains("Windows")) {
            try {
                of = new FileOutputStream(outputfile);
            } catch (FileNotFoundException ex) {
                System.out.println(ColorMessage.error(METHOD_NAME) + "Cannot open file " + outputfile);
                return null;
            }
        }

        // Set the preprocessor path
        String ppPath = m_ppPath;

        if (ppPath == null) {
            if (m_os.contains("Windows")) {
            	// the path for the cl.exe should be configured as required
                ppPath = "C:\\Program Files (x86)\\Microsoft Visual Studio\\2017\\Community\\VC\\Tools\\MSVC\\14.12.25827\\bin\\Hostx64\\x64\\cl.exe";
            } else if (m_os.contains("Linux") || m_os.contains("Mac")) {
                ppPath = "cpp";
            }
        }

        // Add command
        lineCommand.add(ppPath);

        // Add the include paths given as parameters.
        for (int i=0; i < m_includePaths.size(); ++i) {
            if (m_os.contains("Windows")) {
                lineCommand.add(((String) m_includePaths.get(i)).replaceFirst("^-I", "/I"));
            } else if (m_os.contains("Linux") || m_os.contains("Mac")) {
                lineCommand.add(m_includePaths.get(i));
            }
        }

        if (m_os.contains("Windows")) {
            lineCommand.add("/E");
            lineCommand.add("/C");
        }

        // Add input file.
        lineCommand.add(idlFilename);

        if(m_os.contains("Linux") || m_os.contains("Mac")) {
            lineCommand.add(outputfile);
        }

        lineCommandArray = new String[lineCommand.size()];
        lineCommandArray = (String[])lineCommand.toArray(lineCommandArray);

        try {
            Process preprocessor = Runtime.getRuntime().exec(lineCommandArray);
            ProcessOutput errorOutput = new ProcessOutput(preprocessor.getErrorStream(), "ERROR", false, null, true);
            ProcessOutput normalOutput = new ProcessOutput(preprocessor.getInputStream(), "OUTPUT", false, of, true);
            errorOutput.start();
            normalOutput.start();
            exitVal = preprocessor.waitFor();
            errorOutput.join();
            normalOutput.join();
        } catch (Exception e) {
            System.out.println(ColorMessage.error(METHOD_NAME) + "Cannot execute the preprocessor. Reason: " + e.getMessage());
            return null;
        }

        if (of != null) {
            try {
                of.close();
            } catch (IOException e) {
                System.out.println(ColorMessage.error(METHOD_NAME) + "Cannot close file " + outputfile);
            }

        }

        if (exitVal != 0) {
            System.out.println(ColorMessage.error(METHOD_NAME) + "Preprocessor return an error " + exitVal);
            return null;
        }

        return outputfile;
    }


    class ProcessOutput extends Thread
    {
        InputStream is = null;
        OutputStream of = null;
        String type;
        boolean m_check_failures;
        boolean m_found_error = false;
        final String clLine = "#line";
        boolean m_printLine = false;

        ProcessOutput(InputStream is, String type, boolean check_failures, OutputStream of, boolean printLine)
        {
            this.is = is;
            this.type = type;
            m_check_failures = check_failures;
            this.of = of;
            m_printLine = printLine;
        }

        public void run()
        {
            try
            {
                InputStreamReader isr = new InputStreamReader(is);
                BufferedReader br = new BufferedReader(isr);
                String line=null;
                while ( (line = br.readLine()) != null)
                {
                    if(of == null)
                    {
                        if(m_printLine)
                            System.out.println(line);
                    }
                    else
                    {
                        // Sustituir los \\ que pone cl.exe por \
                        if(line.startsWith(clLine))
                        {
                            line = "#" + line.substring(clLine.length());
                            int count = 0;
                            while((count = line.indexOf("\\\\")) != -1)
                            {
                                line = line.substring(0, count) + "\\" + line.substring(count + 2);
                            }
                        }

                        of.write(line.getBytes());
                        of.write('\n');
                    }

                    if(m_check_failures)
                    {
                        if(line.startsWith("Done (failures)"))
                        {
                            m_found_error = true;
                        }
                    }
                }
            }
            catch (IOException ioe)
            {
                ioe.printStackTrace();
            }
        }

        boolean getFoundError()
        {
            return m_found_error;
        }
    }


	//
	//     Main for test
	//
    public static void main(String[] args)
    {
        TestIDLParser parser = new TestIDLParser();

        HashMap<String, ArrayList<String>> argsList = new HashMap<String, ArrayList<String>>();
        String currentOption = "f";
        argsList.put(currentOption, new ArrayList<String>());
        ArrayList<String> current_list = argsList.get(currentOption);
        for (int i = 0; i < args.length; ++i)
        {
            switch(args[i].charAt(0))
            {
                case '-':
                {
                    if (args[i].length() == 2)
                    {
                        switch(args[i].charAt(1))
                        {
                            case 'I': // Include
                                currentOption = "I";
                            break;
                            case 't': // Temporal path
                                currentOption = "t";
                            break;
                            case 'p': // Preprocessor path
                                currentOption = "p";
                            break;
                            default:
                                System.out.println("ERROR: Unknown option: " + args[i]);
                                usage();
                                return;
                        }
                        if (argsList.containsKey(currentOption))
                        {
                            System.out.println("ERROR: Option: previously defined: " + args[i]);
                            usage();
                            return;
                        }
                        else
                        {
                            current_list = new ArrayList<String>();
                            argsList.put(currentOption, current_list);
                        }
                    }
                    else
                    {
                        System.out.println("ERROR: Unknown option: " + args[i]);
                        usage();
                        return;
                    }
                }
                default:
                    current_list.add(args[i]);
                    break;
            }
        }

        if (argsList.containsKey("t"))
        {
            parser.setTemporalPath(argsList.get("t").get(0));
        }

        if (argsList.containsKey("p"))
        {
            parser.setPreprocessorPath(argsList.get("p").get(0));
        }

        if (argsList.containsKey("I"))
        {
            for (String p : argsList.get("I"))
            {
                parser.addIncludePath(p);
            }
        }

        if (argsList.containsKey("f") && argsList.get("f").size() > 0)
        {
            for (String f : argsList.get("f"))
            {
                parser.parse(f);
            }
        }
        else
        {
            System.out.println("ERROR: No input files.");
            usage();
        }
	}

    public static void usage()
    {
        System.out.println("Usage:");
        System.out.println("idlparser <file>+ [-I <include_path>+] [-t <temporal_path>] [-p <preprocessor_path>]");
    }

}
